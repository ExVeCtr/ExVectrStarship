#ifndef FLASHMEM_HPP
#define FLASHMEM_HPP
// Cold code runs from flash and keeps ITCM (RAM1 code) small; empty on the host.
#ifndef FLASHMEM
#ifdef __IMXRT1062__
#define FLASHMEM __attribute__((section(".flashmem")))
#else
#define FLASHMEM
#endif
#endif
#endif
