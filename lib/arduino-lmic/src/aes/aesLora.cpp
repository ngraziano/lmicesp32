/*******************************************************************************
 * Copyright (c) 2016 Matthijs Kooijman
 *
 * LICENSE
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and
 * redistribution.
 *
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/

#include "aesLora.h"
#include "../lmic/bufferpack.h"
#include "../lmic/lorabase.h"
#include <algorithm>

AesLora::AesLora() {
  mbedtls_aes_init(&AESDevCtx);
  mbedtls_aes_init(&nwkSCtx);
  mbedtls_aes_init(&appSCtx);
}

AesLora::~AesLora() {
  mbedtls_aes_free(&AESDevCtx);
  mbedtls_aes_free(&nwkSCtx);
  mbedtls_aes_free(&appSCtx);
}

size_t AesLora::saveState(uint8_t *buffer) {
  uint8_t *orig = buffer;
  memcpy(buffer, &nwkSCtx, sizeof(nwkSCtx));
  buffer += sizeof(nwkSCtx);
  memcpy(buffer, &appSCtx, sizeof(appSCtx));
  buffer += sizeof(appSCtx);
  return buffer - orig;
}

size_t AesLora::loadState(uint8_t *buffer) {
  uint8_t *orig = buffer;
  memcpy(&nwkSCtx, buffer, sizeof(nwkSCtx));
  buffer += sizeof(nwkSCtx);
  memcpy(&appSCtx, buffer, sizeof(appSCtx));
  buffer += sizeof(appSCtx);
  return buffer - orig;
}

void AesLora::setDevKey(uint8_t key[16]) {
  mbedtls_aes_setkey_enc(&AESDevCtx, key, 128);
}
void AesLora::setNetworkSessionKey(uint8_t key[16]) {
  mbedtls_aes_setkey_enc(&nwkSCtx, key, 128);
}
void AesLora::setApplicationSessionKey(uint8_t key[16]) {
  mbedtls_aes_setkey_enc(&appSCtx, key, 128);
}

// Get B0 value in buf
void AesLora::micB0(uint32_t devaddr, uint32_t seqno, uint8_t dndir,
                    uint8_t len, uint8_t buf[AES_BLCK_SIZE]) {
  buf[0] = 0x49;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = dndir;
  wlsbf4(buf + 6, devaddr);
  wlsbf4(buf + 10, seqno);
  buf[14] = 0;
  buf[15] = len;
}

/**
 * Verify MIC
 * len : total length (MIC included)
 */
bool AesLora::verifyMic(uint32_t devaddr, uint32_t seqno, uint8_t dndir,
                        uint8_t *pdu, uint8_t len) {
  uint8_t buf[AES_BLCK_SIZE];
  uint8_t lenWithoutMic = len - MIC_LEN;
  micB0(devaddr, seqno, dndir, lenWithoutMic, buf);
  aes_cmac(pdu, lenWithoutMic, true, &nwkSCtx, buf);
  return std::equal(buf, buf + MIC_LEN, pdu + lenWithoutMic);
}

/**
 * Append MIC
 * len : total length (MIC included)
 */
void AesLora::appendMic(uint32_t devaddr, uint32_t seqno, uint8_t dndir,
                        uint8_t *pdu, uint8_t len) {
  uint8_t buf[AES_BLCK_SIZE];
  uint8_t lenWithoutMic = len - MIC_LEN;
  micB0(devaddr, seqno, dndir, lenWithoutMic, buf);
  aes_cmac(pdu, lenWithoutMic, true, &nwkSCtx, buf);
  // Copy MIC at the end
  std::copy(buf, buf + MIC_LEN, pdu + lenWithoutMic);
}

/**
 * Append join MIC
 * len : total length (MIC included)
 */
void AesLora::appendMic0(uint8_t *pdu, uint8_t len) {
  uint8_t buf[AES_BLCK_SIZE] = {0};
  uint8_t lenWithoutMic = len - MIC_LEN;
  aes_cmac(pdu, lenWithoutMic, false, &AESDevCtx, buf);
  // Copy MIC0 at the end
  std::copy(buf, buf + MIC_LEN, pdu + lenWithoutMic);
}

/**
 * Verify join MIC
 * len : total length (MIC included)
 */
bool AesLora::verifyMic0(uint8_t *pdu, uint8_t len) {
  uint8_t buf[AES_BLCK_SIZE] = {0};
  uint8_t lenWithoutMic = len - MIC_LEN;
  aes_cmac(pdu, lenWithoutMic, 0, &AESDevCtx, buf);
  return std::equal(buf, buf + MIC_LEN, pdu + lenWithoutMic);
}

void AesLora::encrypt(uint8_t *pdu, uint8_t len) {
  // TODO: Check / handle when len is not a multiple of 16
  for (uint8_t i = 0; i < len; i += 16) {
    mbedtls_aes_crypt_ecb(&AESDevCtx, ESP_AES_ENCRYPT, pdu + i, pdu + i);
  }
}

/**
 *  Encrypt data frame payload.
 */
void AesLora::framePayloadEncryption(uint8_t port, uint32_t devaddr,
                                     uint32_t seqno, uint8_t dndir,
                                     uint8_t *payload, uint8_t len) {
  auto ctx = port == 0 ? &nwkSCtx : &appSCtx;
  // Generate
  uint8_t blockAi[AES_BLCK_SIZE];
  blockAi[0] = 1; // mode=cipher
  blockAi[1] = 0;
  blockAi[2] = 0;
  blockAi[3] = 0;
  blockAi[4] = 0;
  blockAi[5] = dndir; // direction (0=up 1=down)
  wlsbf4(blockAi + 6, devaddr);
  wlsbf4(blockAi + 10, seqno);
  blockAi[14] = 0;
  blockAi[15] = 0; // block counter

  while (len) {
    uint8_t blockSi[AES_BLCK_SIZE];

    // Increment the block index byte
    blockAi[15]++;
    // Encrypt the counter block with the selected key
    std::copy(blockAi, blockAi + AES_BLCK_SIZE, blockSi);
    mbedtls_aes_crypt_ecb(ctx, ESP_AES_ENCRYPT, blockSi, blockSi);

    // Xor the payload with the resulting ciphertext
    for (uint8_t i = 0; i < AES_BLCK_SIZE && len > 0; i++, len--, payload++)
      *payload ^= blockSi[i];
  }
}

// Extract session keys
void AesLora::sessKeys(uint16_t devnonce, const uint8_t *artnonce) {
  uint8_t nwkSKey[16];
  uint8_t appSKey[16];
  std::fill(nwkSKey, nwkSKey + 16, 0);
  nwkSKey[0] = 0x01;
  std::copy(artnonce, artnonce + LEN_ARTNONCE + LEN_NETID, nwkSKey + 1);
  wlsbf2(nwkSKey + 1 + LEN_ARTNONCE + LEN_NETID, devnonce);
  std::copy(nwkSKey, nwkSKey + 16, appSKey);
  appSKey[0] = 0x02;

  mbedtls_aes_crypt_ecb(&AESDevCtx, ESP_AES_ENCRYPT, nwkSKey, nwkSKey);
  mbedtls_aes_crypt_ecb(&AESDevCtx, ESP_AES_ENCRYPT, appSKey, appSKey);
  setNetworkSessionKey(nwkSKey);
  setApplicationSessionKey(appSKey);
}

// Shift the given buffer left one bit
static void shift_left(uint8_t *buf, uint8_t len) {
  while (len--) {
    uint8_t next = len ? buf[1] : 0;

    uint8_t val = (*buf << 1);
    if (next & 0x80)
      val |= 1;
    *buf++ = val;
  }
}

// Apply RFC4493 CMAC. If prepend_aux is true,
// result is prepended to the message. result is used as working memory,
// it can be set to "B0" for MIC. The CMAC result is returned in result
// as well.
void AesLora::aes_cmac(const uint8_t *buf, uint8_t len, bool prepend_aux,
                       esp_aes_context *ctx, uint8_t result[AES_BLCK_SIZE]) {

  if (prepend_aux)
    mbedtls_aes_crypt_ecb(ctx, ESP_AES_ENCRYPT, result, result);

  while (len > 0) {
    uint8_t need_padding = 0;
    for (uint8_t i = 0; i < AES_BLCK_SIZE; ++i, ++buf, --len) {
      if (len == 0) {
        // The message is padded with 0x80 and then zeroes.
        // Since zeroes are no-op for xor, we can just skip them
        // and leave AESAUX unchanged for them.
        result[i] ^= 0x80;
        need_padding = 1;
        break;
      }
      result[i] ^= *buf;
    }

    if (len == 0) {
      // Final block, xor with K1 or K2. K1 and K2 are calculated
      // by encrypting the all-zeroes block and then applying some
      // shifts and xor on that.
      uint8_t final_key[16];
      std::fill(final_key, final_key + 16, 0);
      mbedtls_aes_crypt_ecb(ctx, ESP_AES_ENCRYPT, final_key, final_key);

      // Calculate K1
      uint8_t msb = final_key[0] & 0x80;
      shift_left(final_key, sizeof(final_key));
      if (msb)
        final_key[sizeof(final_key) - 1] ^= 0x87;

      // If the final block was not complete, calculate K2 from K1
      if (need_padding) {
        msb = final_key[0] & 0x80;
        shift_left(final_key, sizeof(final_key));
        if (msb)
          final_key[sizeof(final_key) - 1] ^= 0x87;
      }

      // Xor with K1 or K2
      for (uint8_t i = 0; i < sizeof(final_key); ++i)
        result[i] ^= final_key[i];
    }

    mbedtls_aes_crypt_ecb(ctx, ESP_AES_ENCRYPT, result, result);
  }
}
