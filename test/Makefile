CC     = gcc
CFLAGS = -Wall -Wextra -g -DTEST_HOST -I.
LDFLAGS = -lm

TESTS = test_aprs_decode test_ais_decode test_adsb_decode test_wspr_ft8_decode test_cell_decode

all: $(TESTS)
	@echo ""
	@echo "=== Running all tests ==="
	@failed=0; \
	for t in $(TESTS); do \
		echo ""; \
		echo "--- $$t ---"; \
		./$$t; \
		if [ $$? -ne 0 ]; then failed=$$((failed+1)); fi; \
	done; \
	echo ""; \
	if [ $$failed -eq 0 ]; then \
		echo "=== ALL TEST SUITES PASSED ==="; \
	else \
		echo "=== $$failed TEST SUITE(S) FAILED ==="; \
		exit 1; \
	fi

test_%: test_%.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TESTS)

.PHONY: all clean
