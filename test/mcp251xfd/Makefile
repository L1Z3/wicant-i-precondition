DRIVER := ../../components/esp_twai_mcp251xfd
BUILD ?= build
CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -g -O1
CPPFLAGS += -I$(DRIVER) -I$(DRIVER)/vendor
SOURCES := test_core.c fake_mcp2518fd.c $(DRIVER)/mcp251xfd_core.c

.PHONY: all clean sanitize
all: $(BUILD)/test_core $(BUILD)/test_adapter
	./$(BUILD)/test_core
	./$(BUILD)/test_adapter

$(BUILD)/test_core: $(SOURCES) $(BUILD)/vendor.o fake_mcp2518fd.h $(DRIVER)/mcp251xfd_core.h $(DRIVER)/Conf_MCP251XFD.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SOURCES) $(BUILD)/vendor.o -o $@

$(BUILD)/vendor.o: $(DRIVER)/vendor/MCP251XFD.c $(DRIVER)/vendor/MCP251XFD.h $(DRIVER)/Conf_MCP251XFD.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -Wno-sign-compare $(CPPFLAGS) -c $< -o $@

$(BUILD)/test_adapter: test_adapter.c fake_platform.c fake_mcp2518fd.c fake_mcp2518fd.h $(BUILD)/vendor.o $(DRIVER)/esp_twai_mcp251xfd.c $(DRIVER)/mcp251xfd_core.c $(DRIVER)/mcp251xfd_core.h $(wildcard stubs/*.h stubs/*/*.h) $(DRIVER)/include/esp_twai_mcp251xfd.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -Istubs -I$(DRIVER)/include -pthread \
		test_adapter.c fake_platform.c fake_mcp2518fd.c $(DRIVER)/esp_twai_mcp251xfd.c \
		$(DRIVER)/mcp251xfd_core.c $(BUILD)/vendor.o -o $@

sanitize:
	$(MAKE) BUILD=build/sanitize CFLAGS='$(CFLAGS) -fsanitize=address,undefined -fno-omit-frame-pointer' all

clean:
	rm -rf build
