#pragma once

// Classical CAN object: command/address + header + eight payload bytes.
// Larger vendor transfers are split into 16-byte chunks. This stays below the
// ESP32 SPI peripheral's 64-byte non-DMA limit, including initialization.
#define MCP251XFD_TRANS_BUF_SIZE (2 + 8 + 8)
