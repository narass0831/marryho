// SPI 슬레이브 초기화
void SPI_SlaveInit(void) {
    DDRB = (1<<PB6);  // MISO 출력 설정
    SPCR = (1<<SPE);  // SPI 활성화
}

// 데이터 수신 함수
unsigned char SPI_Receive(void) {
    while (!(SPSR & (1<<SPIF)));  // 수신 대기
    return SPDR;  // 수신된 데이터 반환
}