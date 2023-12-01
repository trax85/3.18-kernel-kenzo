KASAN_SANITIZE := n
<<<<<<< HEAD
UBSAN_SANITIZE_kasan.o := n
KCOV_INSTRUMENT := n
=======
>>>>>>> p9x

CFLAGS_REMOVE_kasan.o = -pg
# Function splitter causes unnecessary splits in __asan_load1/__asan_store1
# see: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=63533
CFLAGS_kasan.o := $(call cc-option, -fno-conserve-stack -fno-stack-protector)

<<<<<<< HEAD
obj-y := kasan.o report.o kasan_init.o
=======
obj-y := kasan.o report.o
>>>>>>> p9x
