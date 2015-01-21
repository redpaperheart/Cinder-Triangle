/*
* 
* Copyright (c) 2014, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#pragma once

#include "cinder/Rect.h"

namespace cinder {
template<typename T> class MatrixAffine2;
}

template<typename T>
class TriangleT
{
public:
    typedef glm::detail::tvec2<T, glm::defaultp> Vec2T;
    typedef glm::detail::tvec3<T, glm::defaultp> Vec3T;
    
	TriangleT( const Vec2T& origin = Vec2T(0), const Vec2T& destination = Vec2T(0),
			  const Vec2T& apex = Vec2T(0) );
	static TriangleT<T>	one();
	static TriangleT<T>	zero();

	TriangleT<T>		operator+( const Vec2T& rhs );
	TriangleT<T>		operator+( const TriangleT<T>& rhs );
	void				operator+=( const Vec2T& rhs );
	void				operator+=( const TriangleT<T>& rhs );
	
	TriangleT<T>		operator-( const Vec2T& rhs );
	TriangleT<T>		operator-( const TriangleT<T>& rhs );
	void				operator-=( const Vec2T& rhs );
	void				operator-=( const TriangleT<T>& rhs );
	
	TriangleT<T>		operator*( const T& rhs );
	TriangleT<T>		operator*( const Vec2T& rhs );
	TriangleT<T>		operator*( const TriangleT<T>& rhs );
	void				operator*=( const T& rhs );
	void				operator*=( const Vec2T& rhs );
	void				operator*=( const TriangleT<T>& rhs );
	
	TriangleT<T>		operator/( const T& rhs );
	TriangleT<T>		operator/( const Vec2T& rhs );
	TriangleT<T>		operator/( const TriangleT<T>& rhs );
	void				operator/=( const T& rhs );
	void				operator/=( const Vec2T& rhs );
	void				operator/=( const TriangleT<T>& rhs );

	bool				operator==( const TriangleT<T>& rhs );
	bool				operator!=( const TriangleT<T>& rhs );
	bool				operator<( const TriangleT<T>& rhs );

    T					calcAngle( const Vec2T& point ) const;
	T					calcArea() const;
    Vec2T               calcBarycentricToCartesian( const Vec3T& point ) const;
	ci::RectT<T>        calcBoundingBox() const;
    Vec3T               calcCartesianToBarycentric( const Vec2T& point ) const;
	Vec2T               calcCartesianToPolar( const Vec2T& point ) const;
	Vec2T               calcCentroid() const;
	T					calcHeight() const;
	Vec2T               calcPolarToCartesian( const Vec2T& point ) const;
	const Vec2T         calcSize() const;
	T					calcWidth() const;

	Vec2T&              a();
	const Vec2T&        a() const;
	Vec2T&              b();
	const Vec2T&        b() const;
	Vec2T&              c();
	const Vec2T&        c() const;
	Vec2T&              getApex();
	const Vec2T&        getApex() const;
	Vec2T&              getDestination();
	const Vec2T&        getDestination() const;
	Vec2T&              getOrigin();
	const Vec2T&        getOrigin() const;

	Vec2T               closestPoint( const Vec2T& point ) const;
	bool				contains( const Vec2T& point ) const;
	T					distance( const Vec2T& point ) const;
	T					distanceSquared( const Vec2T& point ) const;
	bool				intersects( const TriangleT<T>& triangle ) const;
	bool				intersection( const Vec2T& point, Vec2T* intersection ) const;

	void				a( const Vec2T& origin );
	void				b( const Vec2T& destination );
	void				c( const Vec2T& apex );
	void				set( const Vec2T& origin, const Vec2T& destination, const Vec2T& apex );
	void				setApex( const Vec2T& apex );
	void				setDestination( const Vec2T& destination );
	void				setOrigin( const Vec2T& origin );
	
    TriangleT<T>		getCentered() const;
    TriangleT<T>		getCenteredFit( const TriangleT<T>& triangle ) const;
	void				include( const Vec2T& point );
	void				include( const std::vector<Vec2T >& points );
	void				include( const TriangleT& triangle );
	void				inflate( const Vec2T& scale );
    TriangleT<T>		inflated( const Vec2T& scale ) const;
	void				offset( const Vec2T& position );
	void				offsetCenterTo( const Vec2T& offset );
	void				rotate( T radians );
    TriangleT<T>		rotated( T radians ) const;
	void				scale( T scale );
    TriangleT<T>		scaled( T scale ) const;
	void				scaleCentered( T scale );
	TriangleT<T>		scaledCentered( T scale ) const;
    TriangleT<T>		transformCopy( const ci::MatrixAffine2<T>& matrix ) const;

	static T			calcAngle( const TriangleT<T>& triangle, const Vec2T& point );
	static T			calcAngle( const Vec2T& a, const Vec2T& b );
	static T			calcArea( const TriangleT<T>& triangle );
	static T			calcArea( const Vec2T& a, const Vec2T& b, const Vec2T& c );
    static Vec2T        calcBarycentricToCartesian( const TriangleT<T>& triangle, const Vec3T& point );
	static ci::RectT<T>	calcBoundingBox( const TriangleT<T>& triangle );
    static Vec3T        calcCartesianToBarycentric( const TriangleT<T>& triangle, const Vec2T& point );
	static Vec2T        calcCartesianToPolar( const Vec2T& a, const Vec2T& b );
	static Vec2T        calcCentroid( const TriangleT<T>& triangle );
	static Vec2T        calcPolarToCartesian( const Vec2T& a, const Vec2T& b );
	static Vec2T        closestPoint( const TriangleT<T>& triangle, const Vec2T& p, int32_t n = 0 );
	static bool			contains( const TriangleT<T>& triangle, const Vec2T& p );
	static T			distance( const TriangleT<T>& triangle, const Vec2T& p );
	static T			distanceSquared( const TriangleT<T>& triangle, const Vec2T& p );
	static TriangleT<T>	getCentered( const TriangleT<T>& triangle );
	static TriangleT<T>	getCenteredFit( const TriangleT<T>& a, const TriangleT<T>& b );
    static TriangleT<T>	inflated( const TriangleT<T>& triangle, const Vec2T& scale );
	static bool			intersection( const TriangleT<T>& triangle, const Vec2T& p, Vec2T* intersection );
	static bool			intersects( const TriangleT<T>& a, const TriangleT<T>& b );
    static TriangleT<T>	rotated( const TriangleT<T>& triangle, T radians );
    static TriangleT<T>	scaled( const TriangleT<T>& triangle, T scale );
    static TriangleT<T>	scaledCentered( const TriangleT<T>& triangle, T scale );
    static TriangleT<T>	transformCopy( const TriangleT<T>& triangle, const ci::MatrixAffine2<T>& matrix );
protected:
	Vec2T               mApex;			// C
	Vec2T               mDestination;	// B
	Vec2T               mOrigin;		// A

	friend				std::ostream& operator<<( std::ostream& out, const TriangleT<T>& triangle );

	///////////////////////////////////////////////////////////////////////////////

	class PointSort
	{
	public:
		PointSort( const Vec2T& point = Vec2T::zero(), T distance = (T)0.0 );
		bool			operator<( const PointSort& rhs ) const;
		bool			operator==( const PointSort& rhs ) const;
		bool			operator!=( const PointSort& rhs ) const;
	private:
		T				mDistance;
		Vec2T           mPoint;
		friend class	TriangleT<T>;
	};
};

///////////////////////////////////////////////////////////////////////////////

typedef TriangleT<float>	Trianglef;
typedef TriangleT<double>	Triangled;

namespace cinder { namespace gl {
template<typename T>
void					drawSolidTriangle( const TriangleT<T>& triangle, bool textureTriangle = false );
template<typename T>
void					drawStrokedTriangle( const TriangleT<T>& triangle );
} }
