OBK_DIR = ../../src
MBEDTLS_DIR = $(OBK_DIR)/../libraries/mbedtls

INCLUDES += -I$(OBK_DIR)/../include
INCLUDES += -I./fixes
INCLUDES += -I$(MBEDTLS_DIR)/include

CCFLAGS += -DPLATFORM_BEKEN -DPLATFORM_BEKEN_NEW

ifeq ($(CFG_SOC_NAME), 1)
CCFLAGS += -DPLATFORM_BK7231T
else ifeq ($(CFG_SOC_NAME), 2)
ifeq ($(SOC_BK7231T), 1)
CCFLAGS += -DPLATFORM_BK7231T
else
CCFLAGS += -DPLATFORM_BK7231U
endif
else ifeq ($(CFG_SOC_NAME), 3)
CCFLAGS += -DPLATFORM_BK7252
else ifeq ($(CFG_SOC_NAME), 5)
CCFLAGS += -DPLATFORM_BK7231N
else ifeq ($(CFG_SOC_NAME), 7)
CCFLAGS += -DPLATFORM_BK7238
else ifeq ($(CFG_SOC_NAME), 8)
CCFLAGS += -DPLATFORM_BK7252N
endif

SRC_C += ./fixes/blank.c

SRC_C += ${MBEDTLS_DIR}/library/ssl_tls.c
SRC_C += ${MBEDTLS_DIR}/library/x509_crt.c
SRC_C += ${MBEDTLS_DIR}/library/entropy.c
SRC_C += ${MBEDTLS_DIR}/library/chachapoly.c
SRC_C += ${MBEDTLS_DIR}/library/ctr_drbg.c
SRC_C += ${MBEDTLS_DIR}/library/ssl_msg.c
SRC_C += ${MBEDTLS_DIR}/library/debug.c
SRC_C += ${MBEDTLS_DIR}/library/md.c
SRC_C += ${MBEDTLS_DIR}/library/sha512.c
SRC_C += ${MBEDTLS_DIR}/library/platform.c
SRC_C += ${MBEDTLS_DIR}/library/platform_util.c
SRC_C += ${MBEDTLS_DIR}/library/sha256.c
SRC_C += ${MBEDTLS_DIR}/library/sha1.c
SRC_C += ${MBEDTLS_DIR}/library/ripemd160.c
SRC_C += ${MBEDTLS_DIR}/library/md5.c
SRC_C += ${MBEDTLS_DIR}/library/cipher.c
SRC_C += ${MBEDTLS_DIR}/library/gcm.c
SRC_C += ${MBEDTLS_DIR}/library/chacha20.c
SRC_C += ${MBEDTLS_DIR}/library/ccm.c
SRC_C += ${MBEDTLS_DIR}/library/constant_time.c
SRC_C += ${MBEDTLS_DIR}/library/aes.c
SRC_C += ${MBEDTLS_DIR}/library/poly1305.c
SRC_C += ${MBEDTLS_DIR}/library/pem.c
SRC_C += ${MBEDTLS_DIR}/library/des.c
SRC_C += ${MBEDTLS_DIR}/library/asn1parse.c
SRC_C += ${MBEDTLS_DIR}/library/base64_mbedtls.c
SRC_C += ${MBEDTLS_DIR}/library/x509.c
SRC_C += ${MBEDTLS_DIR}/library/oid.c
SRC_C += ${MBEDTLS_DIR}/library/pkparse.c
SRC_C += ${MBEDTLS_DIR}/library/ecp.c
SRC_C += ${MBEDTLS_DIR}/library/bignum.c
SRC_C += ${MBEDTLS_DIR}/library/pk.c
SRC_C += ${MBEDTLS_DIR}/library/pk_wrap.c
SRC_C += ${MBEDTLS_DIR}/library/ecdsa.c
SRC_C += ${MBEDTLS_DIR}/library/asn1write.c
SRC_C += ${MBEDTLS_DIR}/library/hmac_drbg.c
SRC_C += ${MBEDTLS_DIR}/library/rsa.c
SRC_C += ${MBEDTLS_DIR}/library/rsa_internal.c
SRC_C += ${MBEDTLS_DIR}/library/ecp_curves.c
SRC_C += ${MBEDTLS_DIR}/library/ssl_ciphersuites.c
SRC_C += ${MBEDTLS_DIR}/library/ecdh.c
SRC_C += ${MBEDTLS_DIR}/library/dhm.c
SRC_C += ${MBEDTLS_DIR}/library/ssl_srv.c
SRC_C += ${MBEDTLS_DIR}/library/cipher_wrap.c
SRC_C += ${MBEDTLS_DIR}/library/arc4.c
SRC_C += ${MBEDTLS_DIR}/library/blowfish.c
SRC_C += ${MBEDTLS_DIR}/library/camellia.c
SRC_C += ${MBEDTLS_DIR}/library/ssl_cli.c
SRC_C += ${MBEDTLS_DIR}/library/pkcs5.c

APP_C += $(OBK_DIR)/../platforms/BK723x/ps.c

APP_C += $(OBK_DIR)/hal/bk7231/hal_adc_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_flashConfig_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_flashVars_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_generic_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_main_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_pins_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_wifi_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_uart_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_ota_bk7231.c

OBK_SRCS = $(OBK_DIR)/
include $(OBK_DIR)/../platforms/obk_main.mk
APP_C += $(OBKM_SRC)
APP_CXX += $(OBKM_SRC_CXX)
CCFLAGS += $(OBK_CFLAGS)

CFLAGS += -DMBEDTLS_CONFIG_FILE=\"tls_config.h\"

# BERRY_MODULEPATH = $(OBK_DIR)/berry/modules
# BERRY_SRCPATH = $(OBK_DIR)/../libraries/berry/src
# include $(OBK_DIR)/../libraries/berry.mk
# APP_C += $(BERRY_SRC_C)
