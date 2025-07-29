#include "ModBussy.h"



#ifdef MB_DEBUG
void byte_to_hex_char_pair_mb(const uint8_t byte_val, char* hex_chars) {
    constexpr char hex_digits[] = "0123456789ABCDEF";
    hex_chars[0] = hex_digits[(byte_val >> 4) & 0x0F];  // Upper nibble
    hex_chars[1] = hex_digits[byte_val & 0x0F];         // Lower nibble
}

void print_byte_array_mb(const uint8_t* bytes, const uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        char hex_chars[3];
        byte_to_hex_char_pair_mb(bytes[i], hex_chars);
        Serial.print(hex_chars);
        Serial.print(" ");
    }
    Serial.println();
}
#endif




// Utility ----
static uint16_t hi(uint16_t v) { return v >> 8; }
static uint16_t lo(uint16_t v) { return v & 0xFF; }

// -----------------------------------------------------------------------------
// Constructor / public API
// -----------------------------------------------------------------------------
ModBussy::ModBussy(const uint16_t tcpPort,
    uint16_t (&coils)[(MB_COIL_BITS+15) / 16],
    uint16_t (&discretes)[(MB_DISCRETE_BITS+15) / 16],
    uint16_t (&holding)[MB_HREGS],
    uint16_t (&input)[MB_IREGS])
        : _coils(coils), _discretes(discretes), _holding(holding), _input(input), _port(tcpPort), _server(tcpPort), _errorCount(0) {}

void ModBussy::begin() {
    _server.begin();
}

void ModBussy::task() {

#ifdef MB_DEBUG
    // Serial.println("Task started");
    // if (_currentClient) {
    //     Serial.print("Current client: ");
    //     Serial.println(_currentClient.remoteIP());
    // } else {
    //     Serial.println("No current client");
    // }

#endif


    // Check if current client is still connected
    if (_currentClient && !_currentClient.connected()) {
#ifdef MB_DEBUG
        Serial.println("Current client disconnected");
#endif
        _currentClient.stop();
        _currentClient = EthernetClient(); // Reset to empty client
    }

    // If no current client, try to accept a new one
    if (!_currentClient || !_currentClient.connected()) {
        EthernetClient newClient = _server.accept();
#ifdef MB_DEBUG
        Serial.print("New client accepted: ");
        Serial.println(newClient.remoteIP());
#endif
        if (newClient && newClient.connected()) {
#ifdef MB_DEBUG
            Serial.print("New client connected: ");
            Serial.println(newClient.remoteIP());
#endif
            _currentClient = newClient;
        }
    }

    // Process requests from current client if available
    if (_currentClient && _currentClient.connected()) {

        // Handle available requests without blocking
        while (_currentClient.available() && _currentClient.connected()) {
            if (!_handleRequest(_currentClient)) {
                // If request handling fails, disconnect the client
#ifdef MB_DEBUG
                Serial.println("Request handling failed, disconnecting client");
#endif
                _currentClient.stop();
                _currentClient = EthernetClient();
                break;
            }
            // Yield for co-operative multitasking
            yield();
        }
    }
}

// -----------------------------------------------------------------------------
// Direct register access helpers
// -----------------------------------------------------------------------------
uint16_t &ModBussy::Hreg(const uint16_t address) const {
    static uint16_t dummy = 0;
    if (address >= MB_HREGS) {
        return dummy; // Out of bounds – user's responsibility
    }
    return _holding[address];
}

uint16_t ModBussy::Ireg(uint16_t address) const {
    return (address < MB_IREGS) ? _input[address] : 0;
}

bool ModBussy::Coil(const uint16_t address) const {
    return _getBit(_coils, address);
}

void ModBussy::Coil(const uint16_t address, const bool v) {
    _setBit(_coils, address, v);
}

bool ModBussy::Discrete(uint16_t address) const {
    return _getBit(_discretes, address);
}

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------
void ModBussy::_flushClient(EthernetClient &c) {
    while (c.available()) c.read();
}

void ModBussy::_sendException(EthernetClient &c, const uint8_t fn, const uint8_t code,
                              const uint16_t tid, const uint8_t unit) {
    uint8_t resp[MBAP_HEADER_SIZE + 2];
    // MBAP header
    resp[0] = hi(tid);
    resp[1] = lo(tid);
    resp[2] = 0; // protocol id hi
    resp[3] = 0; // protocol id lo
    resp[4] = 0; // length hi – unit + 2 bytes PDU
    resp[5] = 3;
    resp[6] = unit;

    // PDU
    resp[7] = fn | 0x80; // set MSB for exception
    resp[8] = code;

    c.write(resp, sizeof(resp));
}

// Get/set bit helpers ---------------------------------------------------------
bool ModBussy::_getBit(const uint16_t *arr, const uint16_t bitAddr) {
    const uint16_t word = bitAddr / 16;
    const uint8_t  bit  = bitAddr % 16;
    return (arr[word] >> bit) & 0x01;
}

void ModBussy::_setBit(uint16_t *arr, const uint16_t bitAddr, const bool val) {
    const uint16_t word = bitAddr / 16;
    const uint8_t  bit  = bitAddr % 16;
    if (val)
        arr[word] |= (1u << bit);
    else
        arr[word] &= ~(1u << bit);
}

// -----------------------------------------------------------------------------
// Core request handler
// -----------------------------------------------------------------------------
bool ModBussy::_handleRequest(EthernetClient &c) {
    if (c.available() < MBAP_HEADER_SIZE) {
        // need more data, but don't fail
        return true;
    }

    uint8_t mbap[MBAP_HEADER_SIZE];
    c.read(mbap, MBAP_HEADER_SIZE);

    const uint16_t tid = (mbap[0] << 8) | mbap[1];
    const uint16_t pid = (mbap[2] << 8) | mbap[3];
    const uint16_t raw_len = (mbap[4] << 8) | mbap[5];
    const uint8_t  uid = mbap[6];

    const uint16_t len = raw_len - 1; // Length includes the unit identifier, which is part of the header

#ifdef MB_DEBUG
    Serial.println("Got request: ");
    Serial.print("tid: ");
    Serial.println(tid);
    Serial.print("pid: ");
    Serial.println(pid);
    Serial.print("len: ");
    Serial.println(len);
    Serial.print("uid: ");
    Serial.println(uid);
#endif

    if (pid != 0 || len == 0 || len > (MAX_PDU_SIZE + 1)) {

        _errorCount++;
        _flushClient(c);
        return false;
    }

    // Wait until full PDU is available (with timeout)
    const uint32_t waitStart = millis();
    while (c.available() < len && c.connected()) {
        if (millis() - waitStart > 100) { // 1 second timeout
#ifdef MB_DEBUG
            Serial.println("Request timed out");
#endif
            return false;
        }
        delay(1);
    }

    if (!c.connected()) {
#ifdef MB_DEBUG
        Serial.println("Client disconnected somehow");
#endif
        return false;
    }

    // TODO: stop reallocating this
    uint8_t  pdu[MAX_PDU_SIZE + 1];
    c.read(pdu, len);
    const uint8_t fn = pdu[0];

    uint8_t  responseBody[MAX_PDU_SIZE + 1];
    uint16_t respLen = 0;
    bool     ok = false;

#ifdef MB_DEBUG
    Serial.print("Got PDU: ");
    Serial.println(fn);
#endif

    switch (fn) {
        case 1: // Read Coils
            ok = _fcReadBits(true, pdu + 1, len - 1, responseBody, respLen);
            break;
        case 2: // Read Discrete Inputs
            ok = _fcReadBits(false, pdu + 1, len - 1, responseBody, respLen);
            break;
        case 3: // Read Holding Registers
            ok = _fcReadRegisters(true, pdu + 1, len - 1, responseBody, respLen);
            break;
        case 4: // Read Input Registers
            ok = _fcReadRegisters(false, pdu + 1, len - 1, responseBody, respLen);
            break;
        case 5: // Write Single Coil
            ok = _fcWriteSingle(true, pdu + 1, len - 1, responseBody, respLen);
            break;
        case 6: // Write Single Register
            ok = _fcWriteSingle(false, pdu + 1, len - 1, responseBody, respLen);
            break;
        case 15: // Write Multiple Coils
            ok = _fcWriteMultipleBits(pdu + 1, len - 1, responseBody, respLen);
            break;
        case 16: // Write Multiple Registers
            ok = _fcWriteMultipleRegisters(pdu + 1, len - 1, responseBody, respLen);
            break;
        default:
            _sendException(c, fn, 1, tid, uid); // illegal function
            _errorCount++;
            return true;
    }

    if (!ok) {
        // Function handler signalled error (addr/quantity)
        _sendException(c, fn, 2, tid, uid); // illegal data address
        _errorCount++;
        return true;
    }

    // Build MBAP header for normal response
    uint8_t mbapResp[MBAP_HEADER_SIZE];
    mbapResp[0] = hi(tid);
    mbapResp[1] = lo(tid);
    mbapResp[2] = 0;
    mbapResp[3] = 0;
    const uint16_t reportedLen = respLen + 2; // + unit id and function code
    mbapResp[4] = hi(reportedLen);
    mbapResp[5] = lo(reportedLen);
    mbapResp[6] = uid;

    // send header + function code + body
#ifdef MB_DEBUG
    Serial.println("Sending response: ");
    Serial.print("Header: ");
    print_byte_array_mb(mbapResp, MBAP_HEADER_SIZE);
    Serial.print("Function code: ");
    Serial.println(fn);
#endif
    c.write(mbapResp, MBAP_HEADER_SIZE);
    c.write(&fn, 1);

    if (respLen) {
#ifdef MB_DEBUG
        Serial.print("Sending response: ");
        print_byte_array_mb(responseBody, respLen);
#endif
        c.write(responseBody, respLen);
    }
    return true;
}

// -----------------------------------------------------------------------------
// Function code processors
// -----------------------------------------------------------------------------
// Read Coils / Discrete inputs --------------------------------------------------
bool ModBussy::_fcReadBits(const bool isCoil, const uint8_t *pdu, const uint16_t len,
                           uint8_t *resp, uint16_t &respLen) const {
#ifdef MB_DEBUG
    Serial.print("Read ");
    Serial.println(isCoil ? "Coils" : "Discrete Inputs");
#endif
    if (len != 4) return false; // malformed
    const uint16_t addr    = (pdu[0] << 8) | pdu[1];
    const uint16_t qtyBits = (pdu[2] << 8) | pdu[3];
#ifdef MB_DEBUG
    Serial.print("Addr: ");
    Serial.println(addr);
    Serial.print("qtyBits: ");
    Serial.println(qtyBits);
#endif
    if (qtyBits == 0 || qtyBits > 2000) return false;

    const uint16_t maxBits = isCoil ? MB_COIL_BITS : MB_DISCRETE_BITS;
    if (addr + qtyBits > maxBits) return false;

#ifdef MB_DEBUG
    Serial.println("Bounds check pass");
#endif

    const uint8_t byteCount = (qtyBits + 7) / 8;
    resp[0] = byteCount;

    for (uint16_t i = 0; i < byteCount; ++i) resp[1 + i] = 0;

    const uint16_t *src = isCoil ? _coils : _discretes;
    for (uint16_t i = 0; i < qtyBits; ++i) {
        // TODO: if starting address allows alignment (n%16==0), don't check every damn bit
        const bool bit = _getBit(src, addr + i);
        if (bit) resp[1 + (i / 8)] |= (1u << (i % 8));
    }

    respLen = 1 + byteCount;
    return true;
}

// Read Holding / Input Registers ----------------------------------------------
bool ModBussy::_fcReadRegisters(const bool isHolding, const uint8_t *pdu, const uint16_t len,
                                uint8_t *resp, uint16_t &respLen) const {
    if (len != 4) return false;
    const uint16_t addr    = (pdu[0] << 8) | pdu[1];
    const uint16_t qtyRegs = (pdu[2] << 8) | pdu[3];
    if (qtyRegs == 0 || qtyRegs > 125) return false;

    const uint16_t maxRegs = isHolding ? MB_HREGS : MB_IREGS;
    if (addr + qtyRegs > maxRegs) return false;

    const uint8_t byteCount = qtyRegs * 2;
    resp[0] = byteCount;

    const uint16_t *src = isHolding ? _holding : _input;
    for (uint16_t i = 0; i < qtyRegs; ++i) {
        const uint16_t val = src[addr + i];
        resp[1 + i * 2]     = hi(val);
        resp[1 + i * 2 + 1] = lo(val);
    }

    respLen = 1 + byteCount;
    return true;
}

// Write Single Coil / Register -------------------------------------------------
bool ModBussy::_fcWriteSingle(const bool isCoil, const uint8_t *pdu, const uint16_t len,
                              uint8_t *resp, uint16_t &respLen) const {
    if (len != 4) return false;
    const uint16_t addr = (pdu[0] << 8) | pdu[1];
    const uint16_t val  = (pdu[2] << 8) | pdu[3];

    if (isCoil) {
        if (addr >= MB_COIL_BITS) return false;
        bool bitVal;
        if (val == 0xFF00) bitVal = true;
        else if (val == 0x0000) bitVal = false;
        else return false;
        _setBit(_coils, addr, bitVal);
    } else {
        if (addr >= MB_HREGS) return false;
        _holding[addr] = val;
    }

    // Echo request back as response body
    memcpy(resp, pdu, 4);
    respLen = 4;
    return true;
}

// Write Multiple Coils ---------------------------------------------------------
bool ModBussy::_fcWriteMultipleBits(const uint8_t *pdu, const uint16_t len,
                                    uint8_t *resp, uint16_t &respLen) const {
    if (len < 5) return false;
    const uint16_t addr     = (pdu[0] << 8) | pdu[1];
    const uint16_t qtyBits  = (pdu[2] << 8) | pdu[3];
    const uint8_t  byteCnt  = pdu[4];
    if (qtyBits == 0 || qtyBits > 1968) return false; // per spec
    if (byteCnt != (qtyBits + 7) / 8) return false;
    if (5 + byteCnt != len) return false;

    if (addr + qtyBits > MB_COIL_BITS) return false;

    const uint8_t *data = pdu + 5;

    for (uint16_t i = 0; i < qtyBits; ++i) {
        const bool bit = (data[i / 8] >> (i % 8)) & 0x01;
        _setBit(_coils, addr + i, bit);
    }

    // Response: echo addr + quantity
    resp[0] = pdu[0];
    resp[1] = pdu[1];
    resp[2] = pdu[2];
    resp[3] = pdu[3];
    respLen = 4;
    return true;
}

// Write Multiple Registers -----------------------------------------------------
bool ModBussy::_fcWriteMultipleRegisters(const uint8_t *pdu, const uint16_t len,
                                         uint8_t *resp, uint16_t &respLen) const {
    if (len < 5) return false;
    const uint16_t addr    = (pdu[0] << 8) | pdu[1];
    const uint16_t qtyRegs = (pdu[2] << 8) | pdu[3];
    const uint8_t  byteCnt = pdu[4];
    if (qtyRegs == 0 || qtyRegs > 123) return false;
    if (byteCnt != qtyRegs * 2) return false;
    if (5 + byteCnt != len) return false;

    if (addr + qtyRegs > MB_HREGS) return false;

    const uint8_t *data = pdu + 5;
    for (uint16_t i = 0; i < qtyRegs; ++i) {
        const uint16_t val = (data[i * 2] << 8) | data[i * 2 + 1];
        _holding[addr + i] = val;
    }

    // Response: echo addr + quantity
    resp[0] = pdu[0];
    resp[1] = pdu[1];
    resp[2] = pdu[2];
    resp[3] = pdu[3];
    respLen = 4;
    return true;
}
