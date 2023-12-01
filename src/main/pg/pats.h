#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct patsConfig_s {
    uint8_t configVersion;
} patsConfig_t;

PG_DECLARE(patsConfig_t, patsConfig);
