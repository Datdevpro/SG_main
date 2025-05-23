# Install script for directory: /home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/lee/.espressif/tools/xtensa-esp32s3-elf/esp-2022r1-11.2.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/build_info.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/compat-2.x.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/config_psa.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/constant_time.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/mbedtls_config.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/private_access.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/psa_util.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/psa" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_builtin_composites.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_builtin_primitives.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_compat.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_config.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_driver_common.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_composites.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_primitives.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_extra.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_platform.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_se_driver.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_sizes.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_struct.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_types.h"
    "/home/lee/work/CP/esp/esp-idf/components/mbedtls/mbedtls/include/psa/crypto_values.h"
    )
endif()

