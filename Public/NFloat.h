#ifndef OC_NFLOAT
#define OC_NFLOAT

#include "OCMath.h"
#include <functional>

template <int N>
struct NFloat
{
	typedef NFloat<N> TVector;
	float data[N];

	NFloat(){}

	explicit NFloat(float k)
	{
		for(int i=0; i<N; ++i)
		{
			data[i] = k;
		}
	}

	template <typename... T>
	NFloat(T... inData)
	: data {inData...}
	{
	}

	NFloat(const TVector& other)
	{
		for(int i=0; i<N; ++i)
		{
			data[i] = other[i];
		}
	}

	TVector& operator=(const TVector& rhs)
	{
		for (int i = 0; i < N; ++i)
		{
			data[i] = rhs[i];
		}
		return *this;
	}

	static bool isNearlyEqual(const TVector& a, const TVector& b, float tolerance = OC_EPSILON)
	{
		bool bNearlyEqual = true;
		for(int i=0; i<N; ++i)
		{
			bNearlyEqual &= ::isNearlyEqual(a[i],b[i]);
		}

		return bNearlyEqual;
	}

	bool isNearlyEqual(const TVector& rhs, float tolerance = OC_EPSILON) const
	{
		return isNearlyEqual(*this, rhs, tolerance);
	}
		
	float operator[](int index) const { return data[index]; }
	float& operator[](int index) { return data[index]; }

	template <typename Lambda>
	static TVector pairwise(const TVector& lhs, const TVector&rhs, Lambda func)
	{
		TVector result;
		for(int i=0; i<N; ++i)
		{
			result[i] = func(lhs[i], rhs[i]);
		}
		return result;
	}

	template <typename Lambda>
	static TVector& pairwise(TVector& lhs, const TVector&rhs, Lambda func)
	{
		for(int i=0; i<N; ++i)
		{
			func(lhs[i], rhs[i]);
		}
		return lhs;
	}

	template <typename Lambda>
	static TVector toMany(const TVector& lhs, float k, Lambda func)
	{
		TVector result;
		for(int i=0; i<N; ++i)
		{
			result[i] = func(lhs[i], k);
		}
		return result;
	}

	template <typename Lambda>
	static TVector& toMany(TVector& lhs, float k, Lambda func)
	{
		for(int i=0; i<N; ++i)
		{
			func(lhs[i], k);
		}
		return lhs;
	}

	TVector operator+(const TVector& rhs) const
	{
		return pairwise(*this, rhs, std::plus<float>());
	}

	TVector& operator+=(const TVector& rhs)
	{
		*this = pairwise(*this, rhs, std::plus<float>());
		return *this;
	}

	TVector operator-(const TVector& rhs) const
	{
		return pairwise(*this, rhs, std::minus<float>());
	}

	TVector& operator-=(const TVector& rhs)
	{
		*this = pairwise(*this, rhs, std::minus<float>());
		return *this;
	}

	TVector operator*(const TVector& rhs) const
	{
		return pairwise(*this, rhs, std::multiplies<float>());
	}

	TVector& operator*=(const TVector& rhs)
	{
		*this = pairwise(*this, rhs, std::multiplies<float>());
		return *this;
	}

	TVector operator/(const TVector& rhs) const
	{
		return pairwise(*this, rhs, std::divides<float>());
	}

	TVector& operator/=(const TVector& rhs)
	{
		*this = pairwise(*this, rhs, std::divides<float>());
		return *this;
	}

	TVector operator*(float k) const
	{
		return toMany(*this, k, std::multiplies<float>());
	}

	TVector& operator*=(float k)
	{
		*this = toMany(*this, k, std::multiplies<float>());
		return *this;
	}

	TVector operator/(float k) const
	{
		return toMany(*this, k, std::divides<float>());
	}

	TVector& operator/=(float k)
	{
		*this = toMany(*this, k, std::divides<float>());
		return *this;
	}

	static float dotProduct(const TVector& a, const TVector& b)
	{
		float sum = float();
		TVector total = a * b;
		for(int i=0; i<N; ++i)
		{
			sum += total[i];
		}
		return sum;
	}

	float dotProduct(const TVector& rhs) const
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

	TVector getNormal() const
	{
		return *this / length();
	}

	TVector& normalize()
	{
		*this /= length();
		return *this;
	}
};

#endif