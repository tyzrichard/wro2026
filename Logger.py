PRINT_LOG = []


def log_print(*args):
    parts = []

    for x in args:
        parts.append(str(x))

    line = " ".join(parts)
    PRINT_LOG.append(line)


def dump_log():
    print("========== SAVED LOG ==========")

    for line in PRINT_LOG:
        print(line)

    print("========== END LOG ==========")


def clear_log():
    del PRINT_LOG[:]