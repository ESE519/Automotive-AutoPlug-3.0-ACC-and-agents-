CC = gcc

BIN = driver_input
SRC = driver_input.c
OBJ = $(SRC:.c=.o)

CFLAGS = -Wall -Wstrict-prototypes -O2

INCS =
LIBS = -lpcan -lwiiuse -lbluetooth -lm

all: $(BIN)

$(BIN): $(OBJ)
	@echo "  [LD]  $@"
	@$(CC) $(CFLAGS) -o $(BIN) $(OBJ) $(LIBS)

.c.o:
	@echo "  [CC]  $@"
	@$(CC) $(CFLAGS) $(INCS) -c $<

clean:
	rm -rf $(BIN) $(OBJ)
