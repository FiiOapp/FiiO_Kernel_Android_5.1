/* http://srecord.sourceforge.net/ */
const uint16_t eprom[] =
{
0x120A, 0x1209, 0x403F, 0xE0D8, 0x903F, 0xE0DA, 0x2416, 0x403F, 0xE0DA,
0x903F, 0xE0DE, 0x2411, 0x403A, 0xE0DE, 0x803A, 0xE0DA, 0x110A, 0x110A,
0x4039, 0xE0DA, 0x493C, 0x4C7F, 0x5F0F, 0x4F1F, 0xE0D8, 0x493D, 0x128F,
0x831A, 0x23F7, 0x403F, 0x0000, 0x903F, 0x0000, 0x2408, 0x403A, 0x0000,
0x3C02, 0x4A3F, 0x128F, 0x903A, 0x0000, 0x23FB, 0x4030, 0xE0C0, 0x40B2,
0x5A80, 0x0120, 0xD3D2, 0x0022, 0xE3D2, 0x0021, 0x40B2, 0xC350, 0x0200,
0x8392, 0x0200, 0x9382, 0x0200, 0x23FB, 0x40B2, 0xC350, 0x0200, 0x8392,
0x0200, 0x9382, 0x0200, 0x27EE, 0x3FFA, 0x4031, 0x0400, 0x12B0, 0xE0CC,
0x930C, 0x2402, 0x12B0, 0xE000, 0x430C, 0x12B0, 0xE058, 0x12B0, 0xE0D0,
0x4C1F, 0x0001, 0x930F, 0x2405, 0x531D, 0x43CD, 0xFFFF, 0x831F, 0x23FB,
0x4130, 0x4134, 0x4135, 0x4136, 0x4137, 0x4138, 0x4139, 0x413A, 0x4130,
0xD032, 0x0010, 0x3FFD, 0x431C, 0x4130, 0x4303, 0x3FFF, 0x0000, 0x0002,
0xE0A2, 0xE0D4, 0x0200, 0xE0C6, 0xE0C6, 0xE0C6, 0xE0C6, 0xE0C6, 0xE0C6,
0xE0C6, 0xE0C6, 0xE0C6, 0xE0C6, 0xE0C6, 0xE088,
};

const uint32_t eprom_address[] =
{
0x0000E000, 0x0000FFE4, 0x0000FFEA, 0x0000FFF8,
};
const uint32_t eprom_length_of_sections[] =
{
0x0000006F, 0x00000002, 0x00000006, 0x00000004,
};
const uint32_t eprom_sections    = 0x00000004;
const uint32_t eprom_termination = 0x00000000;
const uint32_t eprom_start       = 0x0000E000;
const uint32_t eprom_finish      = 0x00010000;
const uint32_t eprom_length      = 0x00002000;

#define EPROM_TERMINATION 0x00000000
#define EPROM_START       0x0000E000
#define EPROM_FINISH      0x00010000
#define EPROM_LENGTH      0x00002000
#define EPROM_SECTIONS    0x00000004