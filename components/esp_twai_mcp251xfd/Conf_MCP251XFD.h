#pragma once

// A complete CAN FD object fits even though this adapter exposes Classical CAN.
#define MCP251XFD_TRANS_BUF_SIZE (2 + 1 + 8 + 64 + 4 + 2)
