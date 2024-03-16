#ifndef CORE_HPP
#define CORE_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --------------------------------------------------------------------------------

using byte = unsigned char;

// --------------------------------------------------------------------------------

#define panic(fmt, ...)\
    do {\
        fprintf(stderr, "\x1b[97m[%s:%d]\033[0m function \x1b[97m'%s'\033[0m \x1b[91mpanicked\033[0m: "\
                fmt "\n",__FILE__, __LINE__, __func__, ##__VA_ARGS__);\
        exit(1);\
    } while (0)

// --------------------------------------------------------------------------------

template<typename Func>
struct Scope_Exit {
    Func function;

    Scope_Exit(Func func) : function(func) {}

    Scope_Exit(const Scope_Exit<Func> &);

    ~Scope_Exit() {
        function();
    }

    Scope_Exit<Func> &operator=(const Scope_Exit<Func> &);
};

struct Scope_Exit_Helper {
    template<typename Func>
    inline Scope_Exit<Func> operator<<(Func func) {
        return func;
    }
};

#define _concat(x, y) x##y
#define concat(x, y) _concat(x, y)

#define defer const auto &concat(_defer_, __LINE__) = Scope_Exit_Helper() << [&]()

// --------------------------------------------------------------------------------

// Lightweight generic dynamic array. Works for POD types only
template<typename T>
struct Array {
    size_t capacity, length;
    T *data;

    static Array<T> make(size_t cap = 2) {
        if (cap < 2) {
            panic("capacity must be at least 2.");
        }

        Array<T> ret{cap, 0, static_cast<T *>(malloc(cap * sizeof(T)))};
        if (!ret.data) {
            panic("could not allocate memory.");
        }

        return ret;
    }

    static Array<T> copy(const Array<T> &other) {
        Array<T> ret{other.capacity, other.length, static_cast<T *>(malloc(other.capacity * sizeof(T)))};
        if (!ret.data) {
            panic("could not allocate memory.");
        }
        memcpy(ret.data, other.data, other.length * sizeof(T));

        return ret;
    }

    void destroy() {
        if (data) {
            free(data);
            data = nullptr;
        }

        capacity = length = 0;
    }

    void reserve(size_t new_cap) {
        if (new_cap <= capacity) {
            panic("new capacity must be greater than current one.");
        }

        data = static_cast<T *>(realloc(data, new_cap * sizeof(T)));
        if (!data) {
            panic("could not allocate memory.");
        }

        capacity = new_cap;
    }

    void push(T val) {
        if (length == capacity) {
            reserve(capacity ? 2 * capacity : 2);
        }

        data[length++] = val;
    }

    void insert(size_t idx, T val) {
        if (idx >= length) {
            panic("index out of bounds.");
        }

        if (length == capacity) {
            reserve(capacity ? 2 * capacity : 2);
        }

        memmove(data + idx + 1, data + idx, (length - idx) * sizeof(T));

        data[idx] = val;
        ++length;
    }

    void pop() {
        if (!length) {
            panic("array is empty.");
        }

        --length;
    }

    void remove(size_t idx) {
        if (idx >= length) {
            panic("index out of bounds.");
        }

        memmove(data + idx, data + idx + 1, (length - 1 - idx) * sizeof(T));

        --length;
    }

    inline void clear() {
        length = 0;
    }

    inline T *begin() {
        return data;
    }

    inline const T *begin() const {
        return data;
    }

    inline T *end() {
        return data + length;
    }

    inline const T *end() const {
        return data + length;
    }

    inline T &operator[](size_t idx) {
        if (idx >= length) {
            panic("index out of bounds.");
        }

        return data[idx];
    }

    inline const T &operator[](size_t idx) const {
        if (idx >= length) {
            panic("index out of bounds.");
        }

        return data[idx];
    }
};

// --------------------------------------------------------------------------------

#endif // CORE_HPP