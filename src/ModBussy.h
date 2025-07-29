#pragma once
#include <Arduino.h>
#include <cstring>
#include <Ethernet.h>

// -----------------------------------------------------------------------------
// ModBussy: A lean, static-storage Modbus-TCP server (slave) implementation.
// Supports function codes 1-4, 5-6, 15-16 (0x10).
// All registers are fixed-size, statically allocated arrays of uint16_t.
// Coils & discrete inputs are stored as bitmasks inside uint16_t words.
// -----------------------------------------------------------------------------

#ifndef MB_COIL_BITS
#define MB_COIL_BITS 256  // Number of coil bits
#endif
#ifndef MB_DISCRETE_BITS
#define MB_DISCRETE_BITS 16  // Number of discrete input bits
#endif
#ifndef MB_HREGS
#define MB_HREGS 128  // Holding registers (uint16_t)
#endif
#ifndef MB_IREGS
#define MB_IREGS 1  // Input registers (uint16_t)
#endif

// #define MB_DEBUG


class ModBussy {
public:
    explicit ModBussy(uint16_t tcpPort,
                     uint16_t (&coils)[(MB_COIL_BITS + 15) / 16],
                     uint16_t (&discretes)[(MB_DISCRETE_BITS + 15) / 16],
                     uint16_t (&holding)[MB_HREGS],
                     uint16_t (&input)[MB_IREGS]);


    // Initialise EthernetServer and begin listening
    void begin();
    // Poll – call regularly from the main loop
    void task();

    // ---------- Direct register access ----------
    uint16_t &Hreg(uint16_t address) const;      // Holding register RW reference
    uint16_t Ireg(uint16_t address) const; // Input register R

    bool Coil(uint16_t address) const;     // Read coil bit
    void Coil(uint16_t address, bool v);   // Write coil bit

    bool Discrete(uint16_t address) const; // Read discrete input bit

    // Error counter (incremented on malformed frames / exceptions)
    uint32_t errorCount() const { return _errorCount; }

    bool hasClient() { return (_currentClient && _currentClient.connected()); }
private:
    // --- Constants ---
    static constexpr uint8_t  MBAP_HEADER_SIZE = 7;
    static constexpr uint16_t MAX_PDU_SIZE    = 253;

    uint16_t (&_coils)[(MB_COIL_BITS + 15) / 16];
    uint16_t (&_discretes)[(MB_DISCRETE_BITS + 15) / 16];
    uint16_t (&_holding)[MB_HREGS];
    uint16_t (&_input)[MB_IREGS];



    // --- Networking ---
    uint16_t       _port;
    EthernetServer _server;
    EthernetClient _currentClient;  // Single persistent client

    // --- Stats ---
    uint32_t _errorCount {0};

    // --- Helpers ---
    static void  _flushClient(EthernetClient &c);
    static void  _sendException(EthernetClient &c, uint8_t fn, uint8_t code,
                         uint16_t tid, uint8_t unit);
    bool  _handleRequest(EthernetClient &c);

    // Function code processors – return true on success, false to emit exception
    bool _fcReadBits   (bool isCoil, const uint8_t *pdu, uint16_t len,
                        uint8_t *resp, uint16_t &respLen) const;
    bool _fcReadRegisters(bool isHolding, const uint8_t *pdu, uint16_t len,
                          uint8_t *resp, uint16_t &respLen) const;
    bool _fcWriteSingle(bool isCoil, const uint8_t *pdu, uint16_t len,
                        uint8_t *resp, uint16_t &respLen) const;
    bool _fcWriteMultipleBits(const uint8_t *pdu, uint16_t len,
                              uint8_t *resp, uint16_t &respLen) const;
    bool _fcWriteMultipleRegisters(const uint8_t *pdu, uint16_t len,
                                   uint8_t *resp, uint16_t &respLen) const;

    // Bit helpers
    static bool  _getBit(const uint16_t *arr, uint16_t bitAddr);
    static void  _setBit(uint16_t *arr, uint16_t bitAddr, bool val);
};