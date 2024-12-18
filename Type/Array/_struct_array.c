// #include "struct_array.h"

// void struct_array_foreach_set_int32(const void * p_buffer, size_t unit_size, size_t length, set_int32_t unit_setter, int32_t value)
// {
//     for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); }
// }

// void struct_array_foreach_set_int16(const void * p_buffer, size_t unit_size, size_t length, set_int16_t unit_setter, int16_t value)
// {
//     for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); }
// }

// void struct_array_foreach_set_int8(const void * p_buffer, size_t unit_size, size_t length, set_int8_t unit_setter, int8_t value)
// {
//     for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); }
// }

// void struct_array_foreach_set_uint32(const void * p_buffer, size_t unit_size, size_t length, set_uint32_t unit_setter, uint32_t value)
// {
//     for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); }
// }

// void struct_array_foreach_set_uint16(const void * p_buffer, size_t unit_size, size_t length, set_uint16_t unit_setter, uint16_t value)
// {
//     for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); }
// }

// void struct_array_foreach_set_uint8(const void * p_buffer, size_t unit_size, size_t length, set_uint8_t unit_setter, uint8_t value)
// {
//     for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); }
// }

// bool struct_array_for_every_set_int32(const void * p_buffer, size_t unit_size, size_t length, try_int32_t unit_try, int32_t value)
// {
//     bool is_every = true;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
//     return is_every;
// }

// bool struct_array_for_every_set_int16(const void * p_buffer, size_t unit_size, size_t length, try_int16_t unit_try, int16_t value)
// {
//     bool is_every = true;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
//     return is_every;
// }

// bool struct_array_for_every_set_int8(const void * p_buffer, size_t unit_size, size_t length, try_int8_t unit_try, int8_t value)
// {
//     bool is_every = true;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
//     return is_every;
// }

// bool struct_array_for_every_set_uint32(const void * p_buffer, size_t unit_size, size_t length, try_uint32_t unit_try, uint32_t value)
// {
//     bool is_every = true;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
//     return is_every;
// }

// bool struct_array_for_every_set_uint16(const void * p_buffer, size_t unit_size, size_t length, try_uint16_t unit_try, uint16_t value)
// {
//     bool is_every = true;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
//     return is_every;
// }

// bool struct_array_for_every_set_uint8(const void * p_buffer, size_t unit_size, size_t length, try_uint8_t unit_try, uint8_t value)
// {
//     bool is_every = true;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
//     return is_every;
// }

// bool struct_array_for_any_set_int32(const void * p_buffer, size_t unit_size, size_t length, try_int32_t unit_try, int32_t value)
// {
//     bool is_any = false;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
//     return is_any;
// }

// bool struct_array_for_any_set_int16(const void * p_buffer, size_t unit_size, size_t length, try_int16_t unit_try, int16_t value)
// {
//     bool is_any = false;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
//     return is_any;
// }

// bool struct_array_for_any_set_int8(const void * p_buffer, size_t unit_size, size_t length, try_int8_t unit_try, int8_t value)
// {
//     bool is_any = false;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
//     return is_any;
// }

// bool struct_array_for_any_set_uint32(const void * p_buffer, size_t unit_size, size_t length, try_uint32_t unit_try, uint32_t value)
// {
//     bool is_any = false;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
//     return is_any;
// }

// bool struct_array_for_any_set_uint16(const void * p_buffer, size_t unit_size, size_t length, try_uint16_t unit_try, uint16_t value)
// {
//     bool is_any = false;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
//     return is_any;
// }

// bool struct_array_for_any_set_uint8(const void * p_buffer, size_t unit_size, size_t length, try_uint8_t unit_try, uint8_t value)
// {
//     bool is_any = false;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
//     return is_any;
// }