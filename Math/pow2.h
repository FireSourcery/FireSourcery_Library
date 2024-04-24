// static inline bool CheckDividerMask(uint32_t num, uint32_t align) { return ((num & (align)) == 0UL); }


//align+mask?
// static inline size_t  align_down(size_t num, size_t align) { return ((num) & -(align)); }
// static inline size_t  align_up(size_t num, size_t align) { return (-(-(num) & -(align))); }
// static inline bool  is_aligned(size_t num, size_t align) { return ((num & (align - 1U)) == 0UL); }
// static inline bool  is_aligned_ptr(const void * ptr, size_t align) { return is_aligned((uintptr_t)ptr, align); }
// static inline bool  is_aligned_mask(size_t num, size_t align) { return ((num & (align)) == 0UL); }