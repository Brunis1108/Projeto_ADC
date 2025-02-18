
// Fontes para A-Z e 0-9. Os caracteres tem 8x8 pixels


static uint8_t font[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Nothing
0x00, 0x7E, 0x81, 0x81, 0x81, 0x81, 0x7E, 0x00, //0
0x00, 0x00, 0x42, 0x7f, 0x40, 0x00, 0x00, 0x00, //1
0x30, 0x49, 0x49, 0x49, 0x49, 0x46, 0x00, 0x00, //2
0x49, 0x49, 0x49, 0x49, 0x49, 0x49, 0x36, 0x00, //3
0x3f, 0x20, 0x20, 0x78, 0x20, 0x20, 0x00, 0x00, //4
0x4f, 0x49, 0x49, 0x49, 0x49, 0x30, 0x00, 0x00, //5
0xFE, 0x89, 0x89, 0x89, 0x89, 0x89, 0x71, 0x00, //6
0x01, 0x01, 0x01, 0x61, 0x31, 0x0d, 0x03, 0x00, //7
0x36, 0x49, 0x49, 0x49, 0x49, 0x49, 0x36, 0x00, //8
0x06, 0x09, 0x09, 0x09, 0x09, 0x09, 0x7f, 0x00, //9
0xFC, 0x12, 0x11, 0x11, 0x11, 0x12, 0xFC, 0x00, //A
0x81, 0xFF, 0x89, 0x89, 0x8D, 0x8A, 0x70, 0x00, //B 
0x7E, 0x81, 0x81, 0x81, 0x81, 0x42, 0xE7, 0x00, //C
0x81, 0xFF, 0x81, 0x81, 0x81, 0x42, 0x3C, 0x00, //D
0xFF, 0x89, 0x89, 0x89, 0x89, 0x81, 0x81, 0x00, //E
0xFF, 0x09, 0x09, 0x09, 0x09, 0x01, 0x01, 0x00, //F
0x7F, 0x81, 0x81, 0x89, 0x89, 0x89, 0x79, 0x00, //G
0xFF, 0x08, 0x08, 0x08, 0x08, 0x08, 0xFF, 0x00, //H
0x00, 0x81, 0x81, 0xFF, 0x81, 0x81, 0x00, 0x00, //I
0x60, 0x90, 0x91, 0x81, 0x7F, 0x01, 0x01, 0x00, //J
0xFF, 0x08, 0x08, 0x14, 0x22, 0x41, 0x80, 0x00, //K
0x81, 0xFF, 0x81, 0x80, 0x80, 0xC0, 0x00, 0x00, //L
0xFF, 0x01, 0x02, 0x0C, 0x02, 0x01, 0xFF, 0x00, //M
0xFF, 0x01, 0x06, 0x0C, 0x30, 0x40, 0xFF, 0x00, //N
0x3C, 0x42, 0x81, 0x81, 0x81, 0x42, 0x3C, 0x00, //O
0xFF, 0x09, 0x09, 0x09, 0x09, 0x09, 0x06, 0x00, //P
0x3e, 0x41, 0x41, 0x49, 0x51, 0x61, 0x7e, 0x00, //Q
0x01, 0xFF, 0x09, 0x19, 0x29, 0x49, 0x86, 0x00, //R
0x46, 0x49, 0x49, 0x49, 0x49, 0x30, 0x00, 0x00, //S
0x01, 0x01, 0x01, 0x7f, 0x01, 0x01, 0x01, 0x00, //T
0x3f, 0x40, 0x40, 0x40, 0x40, 0x40, 0x3f, 0x00, //U
0x0f, 0x10, 0x20, 0x40, 0x20, 0x10, 0x0f, 0x00, //V
0x7f, 0x20, 0x10, 0x08, 0x10, 0x20, 0x7f, 0x00, //W
0x00, 0x41, 0x22, 0x14, 0x14, 0x22, 0x41, 0x00, //X
0x01, 0x02, 0x04, 0x78, 0x04, 0x02, 0x01, 0x00, //Y
0x41, 0x61, 0x59, 0x45, 0x43, 0x41, 0x00, 0x00, //Z 36
0x60, 0x92, 0x92, 0x92, 0x92, 0x92, 0xFC, 0x00, //a 37
0xFF, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, //b
0x7C, 0x82, 0x82, 0x82, 0x82, 0x82, 0x44, 0x00, //c
0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0xFF, 0x00, //d
0x7C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x8C, 0x00, //e
0x08, 0x08, 0xFE, 0x09, 0x09, 0x01, 0x01, 0x00, //f
0x1C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x7E, 0x00, //g
0xFF, 0x08, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, //h
0x00, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00, 0x00, //i
0x80, 0x80, 0x80, 0x7A, 0x00, 0x00, 0x00, 0x00, //j
0x00, 0xFF, 0x10, 0x28, 0x44, 0x80, 0x00, 0x00, //k
0x7F, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, //l
0xFC, 0x02, 0x02, 0x3C, 0x02, 0x02, 0xFC, 0x00, //m
0xFE, 0x02, 0x02, 0x02, 0x02, 0x04, 0xF8, 0x00, //n
0x38, 0x44, 0x82, 0x82, 0x82, 0x44, 0x38, 0x00, //o
0xFE, 0x12, 0x12, 0x12, 0x12, 0x12, 0x0C, 0x00, //p
0x0C, 0x12, 0x12, 0x12, 0x12, 0x12, 0xFE, 0x00, //q
0x00, 0xFE, 0x08, 0x04, 0x02, 0x04, 0x00, 0x00, //r
0x0C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x62, 0x00, //S
0x00, 0x04, 0x04, 0x7F, 0x84, 0x84, 0x80, 0x00, //t
0x7E, 0x80, 0x80, 0x80, 0x80, 0x80, 0x7E, 0x00, //u
0x06, 0x18, 0x60, 0x80, 0x60, 0x18, 0x06, 0x00, //v
0x7E, 0xC0, 0x80, 0x70, 0x80, 0xC0, 0x7E, 0x00, //w
0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82, 0x00, //x
0x00, 0x82, 0x4C, 0x30, 0x0C, 0x02, 0x00, 0x00,//y
0x02, 0xC2, 0xA2, 0x92, 0x92, 0x8A, 0x86, 0x00, //z
0x00, 0x00, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0x00,//:
0x00, 0x00, 0x80, 0xE6, 0x66, 0x00, 0x00, 0x00,//;
0x00, 0x10, 0x28, 0x44, 0x44, 0x00, 0x00, 0x00,//<
0x00, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x00, //=
0x00, 0x00, 0x00, 0x44, 0x44, 0x28, 0x10, 0x00,//>
0x00, 0x00, 0x04, 0x02, 0xB2, 0x0C, 0x00, 0x00, //?
0x7C, 0x82, 0xBA, 0x2A, 0x72, 0x42, 0x7C, 0x00, //@

};
