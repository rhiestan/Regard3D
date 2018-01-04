// Copyright (c) 2013 Steinwurf ApS
// All Rights Reserved
//
// Distributed under the "BSD License". See the accompanying LICENSE.rst file.

#pragma once

#include <memory>

namespace cpuid
{
class cpuinfo
{
public:

    /// Constructor for feature detection with default values
    cpuinfo();

    ~cpuinfo();

    // x86 member functions
    bool has_fpu() const;
    bool has_mmx() const;
    bool has_sse() const;
    bool has_sse2() const;
    bool has_sse3() const;
    bool has_ssse3() const;
    bool has_sse4_1() const;
    bool has_sse4_2() const;
    bool has_pclmulqdq() const;
    bool has_avx() const;
    bool has_avx2() const;
	bool has_popcnt() const;

    // ARM member functions
    bool has_neon() const;

public:

    struct impl;

private:

    std::unique_ptr<impl> m_impl;
};
}
