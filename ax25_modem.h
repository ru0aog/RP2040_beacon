// ax25_modem.h — передающий модем AX.25 300 bd (AFSK Bell 202 / NRZI / HDLC)  
#pragma once  
#include <Arduino.h>  
  
// Подготовка двухтональной сетки Bell 202 вокруг центральной частоты.  
// mark = freq - 500 Гц, space = freq + 500 Гц (инверсия меняет их местами).  
void prepare_ax25_frequencies(uint32_t ax25_freq_hz, bool invert);  
  
// Отправить один UI-frame: dest=AX25_DEST, src=позывной, payload=текст.  
void send_ax25_packet(const String& payload);  
  
// Полная форма: произвольные адреса и payload.  
void send_ax25_frame(const char* dest_call, uint8_t dest_ssid,  
                     const char* src_call,  uint8_t src_ssid,  
                     const uint8_t* info, size_t info_len);  
  
// Self-test: вывести в Serial байты собранного кадра и FCS (без эфира).  
void ax25_self_test();