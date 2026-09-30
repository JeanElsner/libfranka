// Copyright (c) 2026 Jean Elsner
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

/**
 * @file v5.h
 * Wire types of research interface protocol version 5, as spoken by Franka
 * Emika Robot (FER) firmware from system version 4.2.1, the protocol of
 * libfranka 0.9.x.
 *
 * The headers under v5/ are libfranka-common at e6aa0fc210d9, the commit
 * libfranka 0.9.2 pins, unmodified. They are wrapped in franka::protocol::v5 so
 * they can coexist with the current protocol's research_interface namespace.
 */

// The vendored headers include only these. Including them here first, outside
// the namespace, turns those nested includes into no-ops; otherwise the standard
// library would end up declared inside franka::protocol::v5.
#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace franka::protocol::v5 {
#include "v5/rbk_types.h"
#include "v5/service_types.h"
}  // namespace franka::protocol::v5
