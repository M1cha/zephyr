/**
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SLI_CRYPTO_H
#define ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SLI_CRYPTO_H

#include <sl_status.h>
#include <stddef.h>
#include <stdbool.h>

#define SLI_CRYPTO_KEY_LOCATION_PLAINTEXT ((sli_crypto_key_location_t)0x00000000UL)

typedef uint32_t sli_crypto_key_location_t;

typedef struct {
	uint8_t *pointer;
	uint32_t size;
} sli_crypto_key_buffer_t;

typedef struct {
	sli_crypto_key_buffer_t buffer;
	uint32_t key_size;
} sli_crypto_plaintext_key_t;

typedef struct {
	sli_crypto_key_location_t location;
	union {
		sli_crypto_plaintext_key_t plaintext_key;
	} key;
} sli_crypto_descriptor_t;

#define SLI_CRYPTO_DESCRIPTOR_INIT_PLAINTEXT_KEY(ptr, sz)                                          \
	{                                                                                          \
		.location = SLI_CRYPTO_KEY_LOCATION_PLAINTEXT,                                     \
		.key =                                                                             \
			{                                                                          \
				.plaintext_key =                                                   \
					{                                                          \
						.buffer =                                          \
							{                                          \
								.pointer = ptr,                    \
								.size = sz,                        \
							},                                         \
						.key_size = sz,                                    \
					},                                                         \
			},                                                                         \
	}

sl_status_t sli_crypto_ccm_zigbee(sli_crypto_descriptor_t *key_descriptor, bool encrypt,
					 const unsigned char *data_in, unsigned char *data_out,
					 size_t length, const unsigned char *iv,
					 const unsigned char *aad, size_t aad_len,
					 unsigned char *tag, size_t tag_len);

#endif /* ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SLI_CRYPTO_H */

