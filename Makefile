CC       := cc
CFLAGS   := -Iinclude -O3 -ffast-math -march=native -D_REENTRANT
LDFLAGS  := -lm -lSDL2

TARGET    := prismis
SRC       := prismis.c
BUILD_DIR := build
OBJ       := $(BUILD_DIR)/prismis.o

all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR):
	mkdir -p $@

$(OBJ): $(SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
