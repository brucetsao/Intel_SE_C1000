#
# Copyright (c) 2017, Intel Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the Intel Corporation nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

### Variables
DRV_DIR = $(BASE_DIR)/drivers
OBJ_DIR = $(DRV_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)
OBJ_DIRS += $(DRV_DIR)/$(BUILD)/$(SOC)/$(TARGET)

# Include drivers defined in soc/quark_xxx/drivers/drivers.mk
OBJECTS += $(addprefix $(DRV_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/,$(DRV_SOURCES:.c=.o))

### Flags
CFLAGS += -I$(DRV_DIR)
CFLAGS += -I$(DRV_DIR)/include

$(DRV_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/%.o: %.c
	$(call mkdir, $(DRV_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ))
	$(call mkdir, $(DRV_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/$(dir $<))
	$(CC) $(CFLAGS) -c -o $@ $<
