CC = gcc
CFLAGS = -Wall -Wextra -pedantic -std=c11 -g -O2
LDFLAGS = -lm -lpthread

SRC_DIR = src
INC_DIR = include
OBJ_DIR = obj
BIN_DIR = bin

SOURCES = $(SRC_DIR)/main.c \
          $(SRC_DIR)/config.c \
          $(SRC_DIR)/utils.c \
          $(SRC_DIR)/map.c \
          $(SRC_DIR)/ipc.c \
          $(SRC_DIR)/discovery.c \
          $(SRC_DIR)/online_ga.c \
          $(SRC_DIR)/robot.c \
          $(SRC_DIR)/astar.c

OBJECTS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SOURCES))
TARGET = $(BIN_DIR)/online_rescue_ga

.PHONY: all directories run clean clean-ipc help 

all: directories $(TARGET)

directories:
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(BIN_DIR)

$(TARGET): $(OBJECTS)
	@echo "Linking $@..."
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)
	@echo "Build complete: $@"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -I$(INC_DIR) -c $< -o $@

run: all
	@echo "Running GA Rescue Robot with A* Comparison..."
	./$(TARGET) config.txt

clean:
	@echo "Cleaning build files..."
	rm -rf $(OBJ_DIR) $(BIN_DIR)
	rm -f *_results.txt *_config.txt *.log
	@echo "Clean complete"

clean-ipc:
	@echo "Cleaning IPC resources..."
	-ipcrm -M 0x1234 2>/dev/null || true
	-ipcrm -Q 0x5678 2>/dev/null || true
	-ipcrm -S 0x9abc 2>/dev/null || true
	@echo "IPC resources cleaned"

distclean: clean clean-ipc

help:
	@echo "GA Rescue Robot with A* Comparison - Available targets:"
	@echo ""
	@echo "  all         - Build the project (default)"
	@echo "  run         - Run with config_online.txt"
	@echo "  clean       - Remove build files"
	@echo "  clean-ipc   - Remove IPC resources (use after crashes)"
	@echo "  distclean   - Full cleanup"
	@echo "  help        - Show this message"
