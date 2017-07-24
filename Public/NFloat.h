#ifndef OC_FLOAT
#define OC_FLOAT

#include "OCMath.h"
#include <functional>

template <int N>
struct NFloat
{
	typedef NFloat<N> TFloat;
	float data[N];

	NFloat(){}

	explicit NFloat(float k)
	{
		for(int i=0; i<N; ++i)
		{
			data[i] = k;
		}
	}

	NFloat(const TFloat& other)
	{
		for(int i=0; i<N; ++i)
		{
			data[i] = other[i];
		}
	}

	template<typename... T>
	NFloat(T... inData)
	: data {inData...}
	{
	}

	TFloat& operator=(const TFloat& rhs)
	{
		for (int i = 0; i < N; ++i)
		{
			data[i] = rhs[i];
		}
		return *this;
	}

	static bool isNearlyEqual(const TFloat& a, const TFloat& b, float tolerance = OC_BIG_EPSILON)
	{
		bool bNearly = true;
		for(int i=0; i<N; ++i)
		{
			bNearly &= ::isNearlyEqual(a[i],b[i], tolerance);
		}

		return bNearly;
	}

	bool isNearlyEqual(const TFloat& rhs, float tolerance = OC_BIG_EPSILON) const
	{
		return isNearlyEqual(*this, rhs, tolerance);
	}
		
	float operator[](int index) const { return data[index]; }
	float& operator[](int index) { return data[index]; }

	template <typename Lambda>
	static TFloat pairwise(const TFloat& lhs, const TFloat&rhs, Lambda func)
	{
		TFloat result;
		for(int i=0; i<N; ++i)
		{
			result[i] = func(lhs[i], rhs[i]);
		}
		return result;
	}

	template <typename Lambda>
	static TFloat toMany(const TFloat& lhs, float k, Lambda func)
	{
		TFloat result;
		for(int i=0; i<N; ++i)
		{
			result[i] = func(lhs[i], k);
		}
		return result;
	}

	TFloat operator+(const TFloat& rhs) const
	{
		return pairwise(*this, rhs, std::plus<float>());
	}

	TFloat& operator+=(const TFloat& rhs)
	{
		*this = pairwise(*this, rhs, std::plus<float>());
		return *this;
	}

	TFloat operator-(const TFloat& rhs) const
	{
		return pairwise(*this, rhs, std::minus<float>());
	}

	TFloat& operator-=(const TFloat& rhs)
	{
		*this = pairwise(*this, rhs, std::minus<float>());
		return *this;
	}

	TFloat operator*(const TFloat& rhs) const
	{
		return pairwise(*this, rhs, std::multiplies<float>());
	}

	TFloat& operator*=(const TFloat& rhs)
	{
		*this = pairwise(*this, rhs, std::multiplies<float>());
		return *this;
	}

	TFloat operator/(const TFloat& rhs) const
	{
		return pairwise(*this, rhs, std::divides<float>());
	}

	TFloat& operator/=(const TFloat& rhs)
	{
		*this = pairwise(*this, rhs, std::divides<float>());
		return *this;
	}

	TFloat operator*(float k) const
	{
		return toMany(*this, k, std::multiplies<float>());
	}

	TFloat& operator*=(float k)
	{
		*this = toMany(*this, k, std::multiplies<float>());
		return *this;
	}

	TFloat operator/(float k) const
	{
		return toMany(*this, k, std::divides<float>());
	}

	TFloat& operator/=(float k)
	{
		*this = toMany(*this, k, std::divides<float>());
		return *this;
	}

	TFloat operator-() const
	{
		TFloat ret;
		for(int i = 0; i<N; ++i)
		{
			ret[i] = -data[i];
		}
		return ret;
	}

	static float dotProduct(const TFloat& a, const TFloat& b)
	{
		float sum = float();
		TFloat total = a * b;
		for(int i=0; i<N; ++i)
		{
			sum += total[i];
		}
		return sum;
	}

	float dotProduct(const TFloat& rhs) const
	{
		return dotProduct(*this, rhs);
	}

	float length2() const
	{
		return dotProduct(*this, *this);
	}

	float length() const
	{
		return std::sqrt(length2());
	}

	TFloat getNormal() const
	{
		return *this * (1.f / length());
	}

	TFloat& normalize()
	{
		*this *= (1.f / length());
		return *this;
	}
};

template <int N>
inline NFloat<N> operator*(float lhs, const NFloat<N>& rhs)
{
	return rhs * lhs;
}

#endif