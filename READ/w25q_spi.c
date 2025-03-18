#include "w25q_spi.h"
//-------------------------------------------------------------
#define    W25_ENABLE_RESET  0x66
#define    W25_RESET  0x99
#define    W25_READ  0x03
#define    W25_FAST_READ  0x0B
#define    W25_GET_JEDEC_ID  0x9f
//-------------------------------------------------------------
#define cs_set() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
//-------------------------------------------------------------
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
//-------------------------------------------------------------
typedef struct
{
  uint16_t  PageSize;
  uint32_t  PageCount;
  uint32_t  SectorSize;
  uint32_t  SectorCount;
  uint32_t  BlockSize;
  uint32_t  BlockCount;
  uint32_t  NumKB;
  uint8_t   SR1;
  uint8_t   SR2;
  uint8_t   SR3;
  uint8_t   high_cap;
}w25_info_t;
w25_info_t  w25_info;
char str1[30];
uint8_t buf[10];
//-------------------------------------------------------------
void SPI1_Send (uint8_t *dt, uint16_t cnt)
{
  HAL_SPI_Transmit (&hspi1, dt, cnt, 5000);

}
//-------------------------------------------------------------
void SPI1_Recv (uint8_t *dt, uint16_t cnt)
{
  HAL_SPI_Receive (&hspi1, dt, cnt, 5000);
}
//-------------------------------------------------------------
void W25_Reset (void)
{
  cs_set();
  buf[0] = W25_ENABLE_RESET;
  buf[1] = W25_RESET;
  SPI1_Send(buf, 2);
  cs_reset();
}
//-------------------------------------------------------------
void W25_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz)
{
  cs_set();
  buf[0] = W25_READ;
  buf[1] = (addr >> 16) & 0xFF;
  buf[2] = (addr >> 8) & 0xFF;
  buf[3] = addr & 0xFF;
  SPI1_Send(buf, 4);
  SPI1_Recv(data, sz);
  cs_reset();
}
//-------------------------------------------------------------
void W25_Read_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t sz)
{
  if(sz > w25_info.PageSize)
      sz=w25_info.PageSize;
  if((offset+sz) > w25_info.PageSize)
      sz = w25_info.PageSize - offset;
  page_addr = page_addr * w25_info.PageSize + offset;
  buf[0] = W25_FAST_READ;
  if(w25_info.high_cap)
  {
    buf[1] = (page_addr >> 24) & 0xFF;
    buf[2] = (page_addr >> 16) & 0xFF;
    buf[3] = (page_addr >> 8) & 0xFF;
    buf[4] = page_addr & 0xFF;
    buf[5] = 0;
    cs_set();
    SPI1_Send(buf, 6);
  }
  else
  {
    buf[1] = (page_addr >> 16) & 0xFF;
    buf[2] = (page_addr >> 8) & 0xFF;
    buf[3] = page_addr & 0xFF;
    buf[4] = 0;
    cs_set();
    SPI1_Send(buf, 5);
  }
  SPI1_Recv(data, sz);
  cs_reset();
}
//-------------------------------------------------------------
uint32_t W25_Read_ID(void)
{
  uint8_t dt[4];
  buf[0] = W25_GET_JEDEC_ID;
  cs_set();
  SPI1_Send(buf, 1);
  SPI1_Recv(dt,3);
  cs_reset();
  return (dt[0] << 16) | (dt[1] << 8) | dt[2];
}
//-------------------------------------------------------------
void W25_Ini(void)
{
  HAL_Delay(100);
  W25_Reset();
  HAL_Delay(100);
  unsigned int id = W25_Read_ID();
  HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
  sprintf(str1,"ID:0x%X\r\n",id);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  id &= 0x0000ffff;
  w25_info.high_cap = 0;
  switch(id)
  {
    case 0x401A:
      w25_info.high_cap=1;
      w25_info.BlockCount=1024;
      sprintf(str1,"w25qxx Chip: w25q512\r\n");
      break;
    case 0x4019:
      w25_info.high_cap=1;
      w25_info.BlockCount=512;
      sprintf(str1,"w25qxx Chip: w25q256\r\n");
      break;
    case 0x4018:
      w25_info.BlockCount=256;
      sprintf(str1,"w25qxx Chip: w25q128\r\n");
      break;
    case 0x4017:
      w25_info.BlockCount=128;
      sprintf(str1,"w25qxx Chip: w25q64\r\n");
      break;
    case 0x4016:
      w25_info.BlockCount=64;
      sprintf(str1,"w25qxx Chip: w25q32\r\n");
      break;
    case 0x4015:
      w25_info.BlockCount=32;
      sprintf(str1,"w25qxx Chip: w25q16\r\n");
      break;
    case 0x4014:
      w25_info.BlockCount=16;
      sprintf(str1,"w25qxx Chip: w25q80\r\n");
      break;
    case 0x4013:
      w25_info.BlockCount=8;
      sprintf(str1,"w25qxx Chip: w25q40\r\n");
      break;
    case 0x4012:
      w25_info.BlockCount=4;
      sprintf(str1,"w25qxx Chip: w25q20\r\n");
      break;
    case 0x4011:
      w25_info.BlockCount=2;
      sprintf(str1,"w25qxx Chip: w25q10\r\n");
      break;
    default:
      sprintf(str1,"w25qxx Unknown ID\r\n");
      HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
      return;
  }
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  w25_info.PageSize=256;
  w25_info.SectorSize=0x1000;
  w25_info.SectorCount=w25_info.BlockCount*16;
  w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
  w25_info.BlockSize=w25_info.SectorSize*16;
  w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;
  sprintf(str1,"Page Size: %d Bytes\r\n",(unsigned int)w25_info.PageSize);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  sprintf(str1,"Page Count: %u\r\n",(unsigned int)w25_info.PageCount);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  sprintf(str1,"Sector Size: %u Bytes\r\n",(unsigned int)w25_info.SectorSize);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  sprintf(str1,"Sector Count: %u\r\n",(unsigned int)w25_info.SectorCount);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  sprintf(str1,"Block Size: %u Bytes\r\n",(unsigned int)w25_info.BlockSize);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  sprintf(str1,"Block Count: %u\r\n",(unsigned int)w25_info.BlockCount);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  sprintf(str1,"Capacity: %u KB\r\n",(unsigned int)w25_info.NumKB);
  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
}
//-------------------------------------------------------------
