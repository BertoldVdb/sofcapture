// A/B firmware update through esp_ota, checked with a SHA-256.
#include <cstdio>
#include <cstring>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "mbedtls/sha256.h"
#include "sofgps/hal.h"

namespace {

const char* TAG = "ota";

esp_ota_handle_t s_handle;
const esp_partition_t* s_target;
mbedtls_sha256_context s_sha;
bool s_active;

}  // namespace

namespace sofgps::hal {

bool ota_begin(uint32_t size) {
  if (s_active) return false;
  s_target = esp_ota_get_next_update_partition(nullptr);
  if (!s_target) {
    ESP_LOGE(TAG, "no update partition");
    return false;
  }
  if (size > s_target->size) {
    ESP_LOGE(TAG, "image %lu bytes exceeds slot %s (%lu)", (unsigned long)size, s_target->label,
             (unsigned long)s_target->size);
    return false;
  }
  // Also erase the sector after the image, where the signature lives. A stale
  // signature from an earlier upload would let an unsigned image pass.
  const uint32_t sector = 4096;
  uint32_t erase = (size + sector - 1) / sector * sector + sector;
  if (erase > s_target->size) erase = s_target->size;
  esp_err_t err = esp_ota_begin(s_target, erase, &s_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
    return false;
  }
  mbedtls_sha256_init(&s_sha);
  mbedtls_sha256_starts(&s_sha, 0);
  s_active = true;
  ESP_LOGI(TAG, "receiving %lu bytes into %s", (unsigned long)size, s_target->label);
  return true;
}

bool ota_write(const uint8_t* data, size_t len) {
  if (!s_active) return false;
  esp_err_t err = esp_ota_write(s_handle, data, len);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
    return false;
  }
  mbedtls_sha256_update(&s_sha, data, len);
  return true;
}

bool ota_end(const uint8_t expected_sha256[32]) {
  if (!s_active) return false;
  s_active = false;
  uint8_t digest[32];
  mbedtls_sha256_finish(&s_sha, digest);
  mbedtls_sha256_free(&s_sha);
  if (memcmp(digest, expected_sha256, 32) != 0) {
    ESP_LOGE(TAG, "sha256 mismatch");
    esp_ota_abort(s_handle);
    return false;
  }
  esp_err_t err = esp_ota_end(s_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
    return false;
  }
  err = esp_ota_set_boot_partition(s_target);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "image verified, %s selected for the next boot", s_target->label);
  return true;
}

void ota_abort() {
  if (!s_active) return;
  s_active = false;
  mbedtls_sha256_free(&s_sha);
  esp_ota_abort(s_handle);
  ESP_LOGW(TAG, "update aborted");
}

bool ota_pending_verify() {
  esp_ota_img_states_t state;
  const esp_partition_t* running = esp_ota_get_running_partition();
  return esp_ota_get_state_partition(running, &state) == ESP_OK &&
         state == ESP_OTA_IMG_PENDING_VERIFY;
}

void ota_confirm() {
  if (!ota_pending_verify()) return;
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  ESP_LOGI(TAG, "running image confirmed (%s)", esp_err_to_name(err));
}

AppInfo app_info() {
  AppInfo info{};
  const esp_app_desc_t* desc = esp_app_get_description();
  strncpy(info.version, desc->version, sizeof info.version - 1);
  strncpy(info.idf, desc->idf_ver, sizeof info.idf - 1);
  strncpy(info.partition, esp_ota_get_running_partition()->label, sizeof info.partition - 1);
  for (int i = 0; i < 32; ++i) snprintf(info.elf_sha256 + 2 * i, 3, "%02x", desc->app_elf_sha256[i]);
  return info;
}

void restart() { esp_restart(); }

}  // namespace sofgps::hal
