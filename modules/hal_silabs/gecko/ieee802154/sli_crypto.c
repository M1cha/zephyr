#include "sli_crypto.h"

#include <zephyr/sys/__assert.h>
#include <sli_protocol_crypto.h>

sl_status_t sli_crypto_ccm_zigbee(sli_crypto_descriptor_t     *key_descriptor,
                                  bool                        encrypt,
                                  const unsigned char         *data_in,
                                  unsigned char               *data_out,
                                  size_t                      length,
                                  const unsigned char         *iv,
                                  const unsigned char         *aad,
                                  size_t                      aad_len,
                                  unsigned char               *tag,
                                  size_t                      tag_len)
{
  __ASSERT_NO_MSG(key_descriptor != NULL);
  __ASSERT_NO_MSG(data_in != NULL);
  __ASSERT_NO_MSG(iv != NULL);
  __ASSERT_NO_MSG(key_descriptor->location == SLI_CRYPTO_KEY_LOCATION_PLAINTEXT);
  __ASSERT_NO_MSG(key_descriptor->key.plaintext_key.buffer.pointer != NULL);

  return sli_ccm_zigbee(encrypt,
                        data_in,
                        data_out,
                        length,
                        (const unsigned char *)key_descriptor->key.plaintext_key.buffer.pointer,
                        iv,
                        aad,
                        aad_len,
                        tag,
                        tag_len);
}
