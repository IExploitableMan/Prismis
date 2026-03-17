CC       := cc
CFLAGS   := -Iinclude -O3 -ffast-math -march=native -pthread
LDFLAGS  := -lm -lSDL2 -lSDL2_image -lSDL2_ttf

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

run: $(BUILD_DIR)/$(TARGET)
	$(CC) $(CFLAGS) -o $(BUILD_DIR)/$(TARGET) $(SRC) $(LDFLAGS)
	$(BUILD_DIR)/$(TARGET)
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all run clean
