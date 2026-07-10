#ifndef ELCORE_SMEM
#define ELCORE_SMEM


#ifdef __cplusplus
extern "C" {
#endif

//===========================================================================
// elcore_smem
//===========================================================================
typedef struct{
    uint8_t *mem;
    uint32_t memsize;
    uint32_t offset;
} elcore_smem_t;

static inline void elcore_smem_init(elcore_smem_t *cp, uint8_t *mem, uint32_t memsize)
{
    cp->mem = mem;
    cp->offset = 0;
    cp->memsize = memsize;
}

static inline void* elcore_smem_alloc(elcore_smem_t *cp, uint32_t size)
{
    if(size + cp->offset > cp->memsize)
        return NULL;

    void *ptr = &cp->mem[cp->offset];   // address of current offset
    cp->offset += size;                  // bump offset
    return ptr;
}

#define ALIGN4(x) (((x) + 3) & ~3)

static inline void* elcore_smem_alloc_aligned(elcore_smem_t *cp, uint32_t size)
{
    uint32_t offset_aligned = ALIGN4(cp->offset);

    if(offset_aligned + size > cp->memsize)
        return NULL;

    void* ptr = &cp->mem[offset_aligned];
    cp->offset = offset_aligned + size;
    return ptr;
}

#ifdef __cplusplus
}
#endif

#endif