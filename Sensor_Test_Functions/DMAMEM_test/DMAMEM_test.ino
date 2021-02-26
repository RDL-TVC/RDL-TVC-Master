//https://forum.pjrc.com/threads/42565-Simple-DMA-questions-DMAMEM

uint16_t  mem1[100];
DMAMEM uint16_t dmamem[100];
uint16_t  mem2[100];

void setup() {
  uint16_t stackmem[10]; 
  uint8_t *pheapmem;
  while (!Serial) ;
  pheapmem = (uint8_t*)malloc(100);
  Serial.begin(9600);
  Serial.printf("mem1: %lx\n", (uint32_t)mem1);
  Serial.printf("dmamem: %lx\n", (uint32_t)dmamem);
  Serial.printf("mem2: %lx\n", (uint32_t)mem2);
  Serial.printf("stackmem: %lx\n", (uint32_t)stackmem);
  Serial.printf("heapmem: %lx\n", (uint32_t)pheapmem);
}

void loop() {
  // put your main code here, to run repeatedly:

}
