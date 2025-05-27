#include <stdio.h>
#include "port_common.h"

#include "wizchip_conf.h"
#include "wizchip_spi.h"
#include "loopback.h"
#include "socket.h"
#include "wizchip_qspi_pio.h"

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 32) // Send and receive cache size
#define _LOOPBACK_DEBUG_
#define _LOOPBACK_DEBUG_
int32_t send_macraw(uint8_t sn, uint8_t *buf, uint32_t len) ;
int32_t recv_MACRAW(uint8_t sn, uint8_t *buf, uint32_t len); 
int32_t Ethernet_Frame_pass_through(uint8_t sn_source, uint8_t sn_dest,  uint8_t *buf, uint32_t len);
uint32_t get_tcp_seq(const uint8_t *ether_frame)
{
    // Ethernet 헤더는 항상 14바이트
    uint8_t ip_header_len = (ether_frame[14] & 0x0F) * 4;
    uint16_t tcp_offset = 14 + ip_header_len;

    return (ether_frame[tcp_offset + 4] << 24) |
           (ether_frame[tcp_offset + 5] << 16) |
           (ether_frame[tcp_offset + 6] << 8)  |
           (ether_frame[tcp_offset + 7]);
}
static wiz_NetInfo g_net_info_0 =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 11, 2},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
#if _WIZCHIP_ > W5500
        .lla = {0xfe, 0x80, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x02, 0x08, 0xdc, 0xff,
                0xfe, 0x57, 0x57, 0x25},             // Link Local Address
        .gua = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Global Unicast Address
        .sn6 = {0xff, 0xff, 0xff, 0xff,
                0xff, 0xff, 0xff, 0xff,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // IPv6 Prefix
        .gw6 = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Gateway IPv6 Address
        .dns6 = {0x20, 0x01, 0x48, 0x60,
                0x48, 0x60, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x88, 0x88},             // DNS6 server
        .ipmode = NETINFO_STATIC_ALL
#else
        .dhcp = NETINFO_STATIC        
#endif
};

static wiz_NetInfo g_net_info_1 =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x58}, // MAC address
        .ip = {192, 168, 11, 3},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
#if _WIZCHIP_ > W5500
        .lla = {0xfe, 0x80, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x02, 0x08, 0xdc, 0xff,
                0xfe, 0x57, 0x57, 0x24},             // Link Local Address
        .gua = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Global Unicast Address
        .sn6 = {0xff, 0xff, 0xff, 0xff,
                0xff, 0xff, 0xff, 0xff,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // IPv6 Prefix
        .gw6 = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Gateway IPv6 Address
        .dns6 = {0x20, 0x01, 0x48, 0x60,
                0x48, 0x60, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x88, 0x88},             // DNS6 server
        .ipmode = NETINFO_STATIC_ALL
#else
        .dhcp = NETINFO_STATIC        
#endif
};

static uint8_t ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint16_t local_port = 8000; // Local port

static void set_clock_khz(void);
int32_t loopback_tcps_multi_socket(uint8_t *buf, uint16_t port);

int main()
{
    set_clock_khz();

    /*mcu init*/
    stdio_init_all(); // Initialize the main control peripheral.

    sleep_ms(1000 * 3);
    
    printf("==========================================================\n");
    printf("Compiled @ %s, %s\n", __DATE__, __TIME__);
    printf("==========================================================\n");

    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();


    set_cs_select(0);
    wizchip_initialize(); // spi initialization
    wizchip_check();

    network_initialize(g_net_info_0);

    print_network_information(g_net_info_0); // Read back the configuration information and print it
    set_cs_select(1);
    wizchip_initialize(); // spi initialization
    wizchip_check();

    network_initialize(g_net_info_1);

    print_network_information(g_net_info_1); // Read back the configuration information and print it
    set_cs_select(0);


    int retval = socket(0, Sn_MR_MACRAW, 0, 0x20);

    int  retval2 = socket(8, Sn_MR_MACRAW, 0, 0x20 );
    printf ("return  ==  %d \r\n" , retval);
    printf ("return  ==  %d \r\n" , retval2);
    uint8_t recv_buf_0[ETHERNET_BUF_MAX_SIZE*2];
    uint8_t recv_buf_1[ETHERNET_BUF_MAX_SIZE*2];
#if 1
    while (true)
    {          
        Ethernet_Frame_pass_through(0,8 ,recv_buf_0, sizeof(recv_buf_0));
        Ethernet_Frame_pass_through(8,0, recv_buf_1, sizeof(recv_buf_0));
        // sleep_ms(1);

    }
#endif 
#if 0

while (1) {
    uint16_t sz;

    // 1) 소켓 0 → 8 로 완전히 브리지
    while ((sz = getSn_RX_RSR(0)) > 0) {
        int32_t len = recv_MACRAW(0, recv_buf, sz);
        if (len <= 0) break;  // 에러/버퍼 비었으면 탈출

        // 첫 2바이트는 길이 정보
        uint16_t frame_len = (recv_buf[0] << 8) | recv_buf[1];
        // 실제 페이로드만 전송
        send_macraw(8, recv_buf + 2, frame_len - 2);
    }

    // 2) 소켓 8 → 0 로 완전히 브리지
    while ((sz = getSn_RX_RSR(8)) > 0) {
        int32_t len = recv_MACRAW(8, recv_buf, sz);
        if (len <= 0) break;

        uint16_t frame_len = (recv_buf[0] << 8) | recv_buf[1];
        send_macraw(0, recv_buf + 2, frame_len - 2);
    }

    // (선택) 약간의 여유를 주고 싶으면 짧게 대기
    // __delay_ms(1);
}
#endif 
#if 0

while (1) {
    uint16_t rsr, tx_free;
    int32_t  len;
    uint16_t frame_len;

    // 1) 먼저 소켓8(=ACK/요청) 패킷 우선 처리
    rsr = getSn_RX_RSR(8);
    if (rsr > 0) {
        if (rsr > ETHERNET_BUF_MAX_SIZE) rsr = ETHERNET_BUF_MAX_SIZE;
        len = recv_MACRAW(8, recv_buf, rsr);
        if (len > 2) {
            frame_len = (recv_buf[0] << 8) | recv_buf[1];
            // TX 버퍼 여유 체크
            tx_free = getSn_TX_FSR(0);
            if (tx_free >= frame_len) {
                // 실제 이더넷 프레임만 보내기
                send_macraw(0, recv_buf + 2, frame_len - 2);
            }
        }
    }

    // 2) 그다음 소켓0에서 대용량 데이터 처리
    rsr = getSn_RX_RSR(0);
    if (rsr > 0) {
        if (rsr > ETHERNET_BUF_MAX_SIZE) rsr = ETHERNET_BUF_MAX_SIZE;
        len = recv_MACRAW(0, recv_buf, rsr);
        if (len > 2) {
            frame_len = (recv_buf[0] << 8) | recv_buf[1];
            tx_free = getSn_TX_FSR(8);
            if (tx_free >= frame_len) {
                send_macraw(8, recv_buf + 2, frame_len - 2);
            }
        }
    }

    // (선택) 다른 작업과 타이밍 조절이 필요하면 짧게 휴식
    // __delay_ms(1);
}
#endif 

}

static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

#if 1
int32_t Ethernet_Frame_pass_through(uint8_t sn_source, uint8_t sn_dest,  uint8_t *buf, uint32_t len)
{
   volatile uint16_t recvsize = getSn_RX_RSR(sn_source);  // Check the size of the received data available
    while((recvsize) !=0 ){
  
        if (recvsize == 0) return 0; // No data has been received yet
        if (recvsize < len) len = recvsize;    // Limit the requested data to avoid exceeding the buffer size

        wiz_recv_data(sn_source, buf, len);           // Store the received data in the buffer
        // Wait until the command register is released
           
        
        uint16_t freesize = getSn_TX_FSR(sn_dest);
        // Check if the data to be sent exceeds the maximum frame size
        uint32_t len2 =((buf[0] << 8) | buf[1]) -2  ; 
        if (len2 > freesize) len2 = freesize; 
        // Send the data    
        
        wiz_send_data(sn_dest, buf+2, len2);
        
        setSn_CR(sn_dest, Sn_CR_SEND);              // Set the signal for completion of reception
        setSn_CR(sn_source, Sn_CR_RECV);
        
        while (getSn_CR(sn_source)  & Sn_CR_RECV  );
        while (getSn_CR(sn_dest) & Sn_CR_SEND);
        
        recvsize = getSn_RX_RSR(sn_source);  // Check the size of the received data available
        //sleep_ms(1);
    }
   return (int32_t)len;                   // Return the actual size of the received data
}
   #else
int32_t Ethernet_Frame_pass_through(uint8_t sn_source, uint8_t sn_dest,  uint8_t *buf, uint32_t len)
{
   uint16_t recvsize = getSn_RX_RSR(sn_source);  // Check the size of the received data available

   if (recvsize == 0) return SOCK_BUSY;   // No data has been received yet
   if (recvsize < len) len = recvsize;    // Limit the requested data to avoid exceeding the buffer size

   wiz_recv_data(sn_source, buf, len);           // Store the received data in the buffer
                 // Wait until the command register is released

   uint16_t freesize = getSn_TX_FSR(sn_dest);
    // Check if the data to be sent exceeds the maximum frame size
    uint32_t len2 = (buf[0] << 8) | buf[1] ; 
    if (len2 > freesize) len2 = freesize; 
    // Send the data    
   wiz_send_data(sn_dest, buf+2, len2 -2);


  
   setSn_CR(sn_dest, Sn_CR_SEND);              // Set the signal for completion of reception
   setSn_CR(sn_source, Sn_CR_RECV);

   while (getSn_CR(sn_source));
   while (getSn_CR(sn_dest));

   return (int32_t)len;                   // Return the actual size of the received data
}
#endif 
int32_t recv_MACRAW_0(uint8_t sn, uint8_t *buf, uint32_t len)
{
   uint16_t recvsize = getSn_RX_RSR(sn);  // Check the size of the received data available

   if (recvsize == 0) return SOCK_BUSY;   // No data has been received yet

   if (recvsize < len) len = recvsize;    // Limit the requested data to avoid exceeding the buffer size

   wiz_recv_data(sn, buf, len);           // Store the received data in the buffer
   setSn_CR(sn, Sn_CR_RECV);              // Set the signal for completion of reception
   while (getSn_CR(sn));                  // Wait until the command register is released

   return (int32_t)len;                   // Return the actual size of the received data
}
int32_t send_macraw_0(uint8_t sn, uint8_t *buf, uint32_t len) {
    uint16_t freesize = 0;

   // CHECK_SOCKNUM();
   // CHECK_SOCKMODE(Sn_MR_MACRAW);  // Check MACRAW mode
    
    // Check the current socket status
    uint8_t sock_status = getSn_SR(sn);
    if (sock_status != SOCK_MACRAW) {
        close(sn);
        printf("error -1 \r\n") ; 
        return SOCKERR_SOCKSTATUS;   // Error if not in MACRAW mode
    }

    freesize = getSn_TX_FSR(sn);

    // Check if the data to be sent exceeds the maximum frame size
    if (len > freesize) len = freesize;
    // Send the data    
    wiz_send_data(sn, buf, len);
  
    
    // Set the send completion command
    setSn_CR(sn, Sn_CR_SEND);

    // Wait for command processing
    while (getSn_CR(sn));

    return (int32_t)len;   // Return the number of bytes
}

int32_t send_macraw(uint8_t sn, uint8_t *buf, uint32_t len) {
    uint16_t freesize = 0;

    // 소켓 상태 확인
    if (getSn_SR(sn) != SOCK_MACRAW) {
        close(sn);
        printf("error -1 \r\n");
        return SOCKERR_SOCKSTATUS;
    }

    // TX 버퍼 확인 (충분한 공간이 확보될 때까지 대기)
    uint32_t timeout = 0;
    do {
        freesize = getSn_TX_FSR(sn);
        sleep_ms(1);
        if (++timeout > 1000) {
            printf("TX buffer timeout!\n");
            return -2;
        }
    } while (freesize < len);

    // 전송
    wiz_send_data(sn, buf, len);
    setSn_CR(sn, Sn_CR_SEND);

    // SEND 명령 처리 대기
    while (getSn_CR(sn));

    // SEND 완료 플래그 대기
    timeout = 0;
    while ((getSn_IR(sn) & Sn_IR_SENDOK) != Sn_IR_SENDOK) {
        sleep_ms(1);
        if (++timeout > 1000) {
            printf("SEND_OK timeout!\n");
            return -3;
        }
    }

    // SEND_OK 비트 클리어
    setSn_IR(sn, Sn_IR_SENDOK);  // 클리어하려면 1을 써야 함

    return (int32_t)len;
}