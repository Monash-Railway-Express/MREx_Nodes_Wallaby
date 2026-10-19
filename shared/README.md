# Shared headers

Common header file for shared constants.

## Usage

```c
#include "../../../shared/MREx.h"
uint8_t NODE_ID = <NODE NAME>_ID;
// Must NOT be const — CAN_MREx v1.13.0 requires a mutable pointer for the
// FreeRTOS CAN task (xTaskCreatePinnedToCore passes &NODE_ID as pvParameters).
```

`..` represents a parent directory, up one level in the directory tree.