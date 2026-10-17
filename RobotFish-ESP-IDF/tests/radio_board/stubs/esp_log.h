#pragma once
template<class... Args> inline void hostLog(Args&&...) {}
#define ESP_LOGI(...) hostLog(__VA_ARGS__)
#define ESP_LOGW(...) hostLog(__VA_ARGS__)
#define ESP_LOGE(...) hostLog(__VA_ARGS__)
