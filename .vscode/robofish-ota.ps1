[CmdletBinding()]
param(
    [ValidateSet("Check", "Build", "Flash", "Connect", "Upload")]
    [string]$Action = "Build",

    [string]$SerialPort = "AUTO",
    [string]$DeviceHost = "192.168.4.1",
    [string]$OtaUsername = "admin"
)

$ErrorActionPreference = "Stop"

# This project intentionally uses one known ESP-IDF installation. The explicit
# paths also make VS Code tasks independent of the user's interactive shell.
$idfPath = "D:\ESP32\ESP-IDF-v5.3.2"
$idfToolsPath = "D:\ESP32\.espressif"
$pythonExe = "D:\ESP32\.espressif\python_env\idf5.3_py3.13_env\Scripts\python.exe"
$cmakeExe = "D:\CMAKE\bin\cmake.exe"
$ninjaExe = "D:\Ninja\ninja.exe"
$idfPy = Join-Path $idfPath "tools\idf.py"
$idfToolsPy = Join-Path $idfPath "tools\idf_tools.py"
$esptoolPy = Join-Path $idfPath "components\esptool_py\esptool\esptool.py"

$workspace = Split-Path -Parent $PSScriptRoot
$projectPath = Join-Path $workspace "RoboFishPartnerDriverSDK"
$artifactPath = Join-Path $projectPath "build"
$idfBuildPath = Join-Path $artifactPath "idf"
$compiledApplication = Join-Path $idfBuildPath "RoboFishPartnerDriverSDK.bin"
$artifactPattern = "RoboFishPartnerDriverSDK-OTA-*.bin"
$artifactRegex = '^RoboFishPartnerDriverSDK-OTA-[0-9]{8}-[0-9]{6}[.]bin$'
$maxOtaApplicationBytes = 0x330000

function Assert-FileExists {
    param(
        [string]$Path,
        [string]$Description
    )

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "$Description was not found: $Path"
    }
}

function Assert-DirectoryExists {
    param(
        [string]$Path,
        [string]$Description
    )

    if (-not (Test-Path -LiteralPath $Path -PathType Container)) {
        throw "$Description was not found: $Path"
    }
}

function Assert-LastExitCode {
    param([string]$Description)

    if ($LASTEXITCODE -ne 0) {
        throw "$Description failed with exit code $LASTEXITCODE."
    }
}

function Initialize-EspIdfEnvironment {
    if ($script:idfEnvironmentReady) {
        return
    }

    Assert-DirectoryExists -Path $idfPath -Description "ESP-IDF 5.3.2"
    Assert-DirectoryExists -Path $idfToolsPath -Description "ESP-IDF tools folder"
    Assert-FileExists -Path $pythonExe -Description "ESP-IDF Python"
    Assert-FileExists -Path $cmakeExe -Description "CMake"
    Assert-FileExists -Path $ninjaExe -Description "Ninja"
    Assert-FileExists -Path $idfPy -Description "idf.py"
    Assert-FileExists -Path $idfToolsPy -Description "idf_tools.py"

    $env:IDF_PATH = $idfPath
    $env:IDF_TOOLS_PATH = $idfToolsPath
    $env:PYTHONUTF8 = "1"

    # Ask this IDF installation for its exact compiler/debug-tool environment.
    # This is local-only and does not install, update, or download anything.
    $exportLines = @(& $pythonExe $idfToolsPy "export" "--format" "key-value")
    Assert-LastExitCode -Description "ESP-IDF environment export"

    foreach ($lineValue in $exportLines) {
        $line = [string]$lineValue
        $separator = $line.IndexOf('=')
        if ($separator -le 0) {
            continue
        }

        $name = $line.Substring(0, $separator).Trim()
        $value = $line.Substring($separator + 1).Trim()
        if ($name -eq "PATH") {
            $value = $value.Replace("%PATH%", $env:PATH)
        }
        [Environment]::SetEnvironmentVariable($name, $value, "Process")
    }

    # Make the requested standalone CMake and Ninja installations authoritative.
    $fixedToolPaths = @(
        (Split-Path -Parent $cmakeExe),
        (Split-Path -Parent $ninjaExe),
        (Split-Path -Parent $pythonExe)
    )
    $env:PATH = [string]::Join(
        [IO.Path]::PathSeparator,
        [string[]]($fixedToolPaths + @($env:PATH))
    )

    $script:idfEnvironmentReady = $true
}

function Invoke-EspIdf {
    param([string[]]$Arguments)

    Initialize-EspIdfEnvironment
    Push-Location -LiteralPath $projectPath
    try {
        & $pythonExe $idfPy @Arguments
        Assert-LastExitCode -Description "idf.py"
    } finally {
        Pop-Location
    }
}

function Assert-BuildInputs {
    Assert-DirectoryExists -Path $projectPath -Description "RoboFish ESP-IDF project"
    Assert-FileExists -Path (Join-Path $projectPath "CMakeLists.txt") -Description "ESP-IDF project CMakeLists.txt"
}

function Read-PlainTextSecret {
    param([string]$Prompt)

    $secureValue = Read-Host $Prompt -AsSecureString
    $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secureValue)
    try {
        $plainValue = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($bstr)
    } finally {
        [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr)
    }

    if ([string]::IsNullOrEmpty($plainValue)) {
        throw "A password is required."
    }
    return $plainValue
}

function Get-DeviceBaseUri {
    $candidate = $DeviceHost.Trim()
    if (-not $candidate) {
        throw "Fish AP address is required. The firmware default is 192.168.4.1."
    }
    if ($candidate -notmatch '^https?://') {
        $candidate = "http://$candidate"
    }

    $uri = $null
    if (-not [Uri]::TryCreate($candidate, [UriKind]::Absolute, [ref]$uri)) {
        throw "Invalid device address: $DeviceHost"
    }
    if ($uri.Scheme -ne "http") {
        throw "The current RoboFish OTA service supports HTTP only."
    }

    $builder = New-Object UriBuilder($uri)
    $builder.Path = "/"
    $builder.Query = ""
    $builder.Fragment = ""
    return $builder.Uri.AbsoluteUri.TrimEnd('/')
}

function Get-LatestOtaArtifact {
    if (-not (Test-Path -LiteralPath $artifactPath -PathType Container)) {
        throw "Build folder was not found. Run the build task first."
    }

    $artifacts = @(
        Get-ChildItem -LiteralPath $artifactPath -File -Filter $artifactPattern |
            Where-Object { $_.Name -match $artifactRegex } |
            Sort-Object Name -Descending
    )
    if ($artifacts.Count -eq 0) {
        throw "No timestamped OTA image was found. Run the build task first."
    }

    $artifact = $artifacts[0]
    if ($artifact.Length -le 0) {
        throw "The OTA image is empty: $($artifact.FullName)"
    }
    if ($artifact.Length -gt $maxOtaApplicationBytes) {
        throw "The OTA image exceeds the 0x330000-byte application partition: $($artifact.Length) bytes"
    }
    return $artifact
}

function Select-WiredApplicationImage {
    Add-Type -AssemblyName System.Windows.Forms

    $dialog = New-Object System.Windows.Forms.OpenFileDialog
    try {
        $dialog.Title = "选择要有线烧录的 ESP32-S3 应用固件"
        $dialog.Filter = "ESP32 application image (*.bin)|*.bin"
        $dialog.CheckFileExists = $true
        $dialog.Multiselect = $false
        if (Test-Path -LiteralPath $artifactPath -PathType Container) {
            $dialog.InitialDirectory = $artifactPath
        }
        if ($dialog.ShowDialog() -ne [System.Windows.Forms.DialogResult]::OK) {
            throw "Wired flash cancelled. No firmware was written."
        }

        $image = Get-Item -LiteralPath $dialog.FileName
        if ($image.Length -le 0 -or $image.Length -gt $maxOtaApplicationBytes) {
            throw "Selected application image has an invalid size: $($image.Length) bytes"
        }
        return $image
    } finally {
        $dialog.Dispose()
    }
}

function Get-DeviceStatus {
    param([string]$BaseUri)

    $response = Invoke-WebRequest -UseBasicParsing -Uri "$BaseUri/get_ip" -Method Get -TimeoutSec 10
    Write-Host "[RoboFish] Device identity HTTP $([int]$response.StatusCode):"
    Write-Host $response.Content
    $status = $response.Content | ConvertFrom-Json
    if ($status.version -ne "4.0.0-partner") {
        throw "The HTTP endpoint is not RoboFish Partner firmware 4.0.0-partner."
    }
    if ([string]$status.ap_ip -ne "192.168.4.1") {
        throw "The device did not report the expected RoboFish AP address 192.168.4.1."
    }
    return $status
}

function Get-CurrentWifiSsid {
    if (-not (Get-Command "netsh.exe" -ErrorAction SilentlyContinue)) {
        throw "Windows netsh.exe was not found; the current Wi-Fi cannot be verified."
    }

    $interfaceOutput = @(& netsh.exe wlan show interfaces 2>&1)
    if ($LASTEXITCODE -ne 0) {
        throw "Windows failed to read the current Wi-Fi connection: $($interfaceOutput -join ' ')"
    }

    foreach ($line in $interfaceOutput) {
        if ([string]$line -match '^\s*SSID\s*:\s*(.+?)\s*$') {
            return $Matches[1]
        }
    }
    throw "No connected Wi-Fi SSID was found. Connect Windows to the target fish AP first."
}

function Assert-CurrentFishAccessPoint {
    $currentSsid = Get-CurrentWifiSsid
    if ($currentSsid -notmatch '^RoboFish-[A-Za-z0-9._-]+$') {
        throw "OTA refused: current Wi-Fi is '$currentSsid', not a RoboFish-* AP. Connect Windows to the fish you intend to flash."
    }

    $baseUri = Get-DeviceBaseUri
    Write-Host "[RoboFish] Current Windows Wi-Fi target: $currentSsid"
    $status = Get-DeviceStatus -BaseUri $baseUri
    Write-Host "[RoboFish] Current fish AP verified: $currentSsid ($($status.ap_ip))"
    $status | Add-Member -NotePropertyName current_ssid -NotePropertyValue $currentSsid -Force
    return $status
}

function Send-OtaImage {
    Add-Type -AssemblyName System.Net.Http

    # Never change Wi-Fi here. Only the RoboFish AP currently selected by the
    # user in Windows is eligible for OTA.
    $artifact = Get-LatestOtaArtifact
    $hash = Get-FileHash -Algorithm SHA256 -LiteralPath $artifact.FullName
    $baseUri = Get-DeviceBaseUri
    $status = Assert-CurrentFishAccessPoint

    Write-Host "[RoboFish] Selected latest OTA image:"
    Write-Host $artifact.FullName
    Write-Host "[RoboFish] Size: $($artifact.Length) bytes"
    Write-Host "[RoboFish] SHA-256: $($hash.Hash.ToLowerInvariant())"
    Write-Host "[RoboFish] OTA target fish SSID: $($status.current_ssid)"
    Write-Host "[RoboFish] OTA target fish AP IP: $($status.ap_ip)"

    if ([string]::IsNullOrWhiteSpace($OtaUsername)) {
        throw "OTA username is required."
    }
    $otaPassword = Read-PlainTextSecret "Web OTA password"
    $credentialBytes = [Text.Encoding]::UTF8.GetBytes("$OtaUsername`:$otaPassword")
    $basicToken = [Convert]::ToBase64String($credentialBytes)
    [Array]::Clear($credentialBytes, 0, $credentialBytes.Length)
    $credentialBytes = $null
    $otaPassword = $null

    $handler = New-Object Net.Http.HttpClientHandler
    $client = $null

    try {
        $handler.UseProxy = $false
        $client = New-Object Net.Http.HttpClient($handler)
        $client.Timeout = [TimeSpan]::FromSeconds(180)

        $authRequest = $null
        $authResponse = $null
        try {
            $authRequest = New-Object Net.Http.HttpRequestMessage([Net.Http.HttpMethod]::Get, "$baseUri/update")
            $authRequest.Headers.Authorization = New-Object Net.Http.Headers.AuthenticationHeaderValue("Basic", $basicToken)
            $authResponse = $client.SendAsync($authRequest).GetAwaiter().GetResult()
            $authBody = $authResponse.Content.ReadAsStringAsync().GetAwaiter().GetResult()
            Write-Host "[RoboFish] Authentication check HTTP $([int]$authResponse.StatusCode)."
            if ([int]$authResponse.StatusCode -eq 401) {
                throw "OTA authentication failed. The firmware was not uploaded."
            }
            if (-not $authResponse.IsSuccessStatusCode) {
                throw "OTA endpoint check failed: HTTP $([int]$authResponse.StatusCode) $authBody"
            }
        } finally {
            if ($authResponse) {
                $authResponse.Dispose()
            }
            if ($authRequest) {
                $authRequest.Dispose()
            }
        }

        $confirmation = Read-Host "Type UPLOAD to stop actuators, write firmware, and reboot"
        if ($confirmation -cne "UPLOAD") {
            throw "Upload cancelled. No firmware was written."
        }

        $uploadRequest = $null
        $uploadResponse = $null
        $multipart = $null
        $fileStream = $null
        $fileContent = $null
        try {
            $uploadRequest = New-Object Net.Http.HttpRequestMessage([Net.Http.HttpMethod]::Post, "$baseUri/update")
            $uploadRequest.Headers.Authorization = New-Object Net.Http.Headers.AuthenticationHeaderValue("Basic", $basicToken)
            $multipart = New-Object Net.Http.MultipartFormDataContent
            $fileStream = [IO.File]::OpenRead($artifact.FullName)
            $fileContent = New-Object Net.Http.StreamContent($fileStream)
            $fileStream = $null
            $fileContent.Headers.ContentType = New-Object Net.Http.Headers.MediaTypeHeaderValue("application/octet-stream")
            $multipart.Add($fileContent, "firmware", $artifact.Name)
            $fileContent = $null
            $uploadRequest.Content = $multipart
            $multipart = $null

            Write-Host "[RoboFish] Uploading firmware. Do not power off the device..."
            $uploadResponse = $client.SendAsync($uploadRequest).GetAwaiter().GetResult()
            $uploadBody = $uploadResponse.Content.ReadAsStringAsync().GetAwaiter().GetResult()
            Write-Host "[RoboFish] Device feedback HTTP $([int]$uploadResponse.StatusCode):"
            Write-Host $uploadBody
            if (-not $uploadResponse.IsSuccessStatusCode) {
                throw "Web OTA failed. The device reported HTTP $([int]$uploadResponse.StatusCode)."
            }
            Write-Host "[RoboFish] OTA accepted. The device is rebooting."
        } finally {
            if ($uploadResponse) {
                $uploadResponse.Dispose()
            }
            if ($uploadRequest) {
                $uploadRequest.Dispose()
            }
            if ($multipart) {
                $multipart.Dispose()
            }
            if ($fileContent) {
                $fileContent.Dispose()
            }
            if ($fileStream) {
                $fileStream.Dispose()
            }
        }
    } finally {
        $basicToken = $null
        if ($client) {
            $client.Dispose()
        } else {
            $handler.Dispose()
        }
    }
}

switch ($Action) {
    "Check" {
        Assert-BuildInputs
        Initialize-EspIdfEnvironment

        Write-Host "[RoboFish] ESP-IDF: $idfPath"
        & $pythonExe $idfPy --version
        Assert-LastExitCode -Description "idf.py version check"

        Write-Host "[RoboFish] Python: $pythonExe"
        & $pythonExe --version
        Assert-LastExitCode -Description "Python version check"

        Write-Host "[RoboFish] CMake: $cmakeExe"
        & $cmakeExe --version
        Assert-LastExitCode -Description "CMake version check"

        Write-Host "[RoboFish] Ninja: $ninjaExe"
        & $ninjaExe --version
        Assert-LastExitCode -Description "Ninja version check"

        Write-Host "[RoboFish] Target: ESP32-S3"
        Write-Host "[RoboFish] Project: $projectPath"
        Write-Host "[RoboFish] Build folder: $idfBuildPath"
        Write-Host "[RoboFish] ESP-IDF environment check passed."
    }

    "Build" {
        Assert-BuildInputs
        New-Item -ItemType Directory -Path $artifactPath -Force | Out-Null

        Write-Host "[RoboFish] Building the ESP-IDF OTA application image..."
        Invoke-EspIdf -Arguments @("-B", $idfBuildPath, "build")
        Assert-FileExists -Path $compiledApplication -Description "Compiled ESP-IDF application image"

        $compiledFile = Get-Item -LiteralPath $compiledApplication
        if ($compiledFile.Length -le 0) {
            throw "The compiled application image is empty: $compiledApplication"
        }
        if ($compiledFile.Length -gt $maxOtaApplicationBytes) {
            throw "Compiled image exceeds the 0x330000-byte OTA application partition: $($compiledFile.Length) bytes"
        }

        $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
        $otaFile = Join-Path $artifactPath "RoboFishPartnerDriverSDK-OTA-$timestamp.bin"
        Copy-Item -LiteralPath $compiledApplication -Destination $otaFile

        $artifact = Get-Item -LiteralPath $otaFile
        $hash = Get-FileHash -Algorithm SHA256 -LiteralPath $otaFile

        Write-Host ""
        Write-Host "============================================================"
        Write-Host "Timestamped Web OTA application image:"
        Write-Host $artifact.FullName
        Write-Host "Size: $($artifact.Length) bytes"
        Write-Host "SHA-256: $($hash.Hash.ToLowerInvariant())"
        Write-Host "Upload this timestamped OTA application image only."
        Write-Host "============================================================"
    }

    "Flash" {
        Assert-BuildInputs
        Initialize-EspIdfEnvironment
        Assert-FileExists -Path $esptoolPy -Description "ESP-IDF esptool.py"

        $bootloaderImage = Join-Path $idfBuildPath "bootloader\bootloader.bin"
        $partitionImage = Join-Path $idfBuildPath "partition_table\partition-table.bin"
        $otaDataImage = Join-Path $idfBuildPath "ota_data_initial.bin"
        Assert-FileExists -Path $bootloaderImage -Description "Bootloader image; run the build task first"
        Assert-FileExists -Path $partitionImage -Description "Partition table image; run the build task first"
        Assert-FileExists -Path $otaDataImage -Description "OTA initial data; run the build task first"

        $applicationImage = Select-WiredApplicationImage
        $applicationHash = Get-FileHash -Algorithm SHA256 -LiteralPath $applicationImage.FullName
        $availablePorts = @([IO.Ports.SerialPort]::GetPortNames())
        if ($availablePorts.Count -eq 0) {
            throw "No serial port was found. Connect the ESP32-S3 with a USB data cable."
        }

        $serialArguments = @()
        if ($SerialPort -eq "AUTO") {
            Write-Host "[RoboFish] Available serial ports: $($availablePorts -join ', ')"
            if ($availablePorts.Count -eq 1) {
                $serialArguments = @("-p", $availablePorts[0])
                Write-Host "[RoboFish] Automatically selected $($availablePorts[0])."
            } else {
                Write-Host "[RoboFish] esptool will probe the available ports for the ESP32-S3."
            }
        } elseif ($SerialPort -match '^COM[0-9]+$') {
            if ($SerialPort -notin $availablePorts) {
                throw "Serial port $SerialPort was not found. Available ports: $($availablePorts -join ', ')"
            }
            $serialArguments = @("-p", $SerialPort)
        } else {
            throw "Invalid ESP32 serial port: $SerialPort"
        }

        Write-Host "[RoboFish] Selected application image: $($applicationImage.FullName)"
        Write-Host "[RoboFish] Size: $($applicationImage.Length) bytes"
        Write-Host "[RoboFish] SHA-256: $($applicationHash.Hash.ToLowerInvariant())"
        Write-Host "[RoboFish] Flashing bootloader, partition table, OTA data, and selected application ..."
        & $pythonExe $esptoolPy --chip esp32s3 @serialArguments -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0 $bootloaderImage 0x8000 $partitionImage 0xe000 $otaDataImage 0x10000 $applicationImage.FullName
        Assert-LastExitCode -Description "Wired flash"
        Write-Host "[RoboFish] Wired flash completed successfully."
    }

    "Connect" {
        $null = Assert-CurrentFishAccessPoint
    }

    "Upload" {
        Send-OtaImage
    }
}
