#include "platform.h"

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pats.h"

PG_REGISTER_WITH_RESET_TEMPLATE(patsConfig_t, patsConfig, PG_PATS_CONFIG, 0);

PG_RESET_TEMPLATE(patsConfig_t, patsConfig,
    .configVersion = 0xFF,
);
