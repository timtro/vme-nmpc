#ifndef _LINEAR_H_
#define _LINEAR_H_

#include<iostream>
#include<fstream>
#include<vector>
#include<iterator>
#include<cmath>
#include<limits>
#include<utility>
#include<type_traits>

/**
 * When applicable, we recommend using the typedefs that are declared below.
 * (point2d, point3d, Color, etc.)
 */

template <typename R,int n>
class Point {
    R v[n];
public:
    static const unsigned size = n;
    Point() {}
    explicit Point(std::initializer_list<double> p) {
        auto iter = p.begin();
        for( unsigned i=0; i<size && i<p.size(); ++i) v[i] = R(*iter++);
    }
    template <typename R2>
    explicit Point(const Point<R2,n>& p) {
        for( unsigned i=0; i<size; ++i) v[i] = R(p[i]);
    }
    explicit Point(R val) { for( unsigned i=0; i < size; ++i) v[i] = val; }
    R& operator[](int i) { return v[i]; }
    const R& operator[](int i) const { return v[i]; }
    typedef R value_type;
    template <typename R2> Point<R,n>& operator=( const Point<R2,n>& b) {
        for( unsigned i=0; i < size; ++i) v[i] = R(b[i]);
        return *this;
    }
};

template <typename R,int n>
class SquareMatrix {
    R v[n][n];
public:
    SquareMatrix() {}

    SquareMatrix(std::initializer_list<std::initializer_list<R>> p) {
        auto iter = p.begin();
        for( unsigned j=0; j<n && j<p.size(); ++j,++iter) {
            auto iter2 = iter->begin();
            for( unsigned i=0; i<n && i < iter->size(); ++i,++iter2)
                v[j][i] = R(*iter2);
        }
    }

    Point<R*,n> getSwappableRows() {
        Point<R*,n> rows;
        for( int i=0; i < n; ++i) rows[i] = v[i];
        return rows;
    }

    R* operator[]( int i) {
        return v[i];
    }

    const R* operator[]( int i) const {
        return v[i];
    }

    template <typename S>
    auto operator*( const Point<S,n>& X) const ->
        Point<decltype(v[0][0]*X[0]),n>
    {
        Point<decltype(v[0][0]*X[0]),n> Y;
        for( int j=0; j < n; ++j) {
            Y[j] = v[j][0] * X[0];
            for( int i=1; i < n; ++i)
                Y[j] += v[j][i] * X[i];
        }
        return Y;
    }

    template <typename S>
    auto operator*( const SquareMatrix<S,n>& X) const ->
        SquareMatrix<decltype(v[0][0]*X[0][0]),n>
    {
        SquareMatrix<decltype(v[0][0]*X[0][0]),n> Y;
        for( int k=0; k < n; ++k) {
            for( int j=0; j < n; ++j) {
                Y[j][k] = v[j][0] * X[0][k];
                for( int i=1; i < n; ++i)
                    Y[j][k] += v[j][i] * X[i][k];
            }
        }
        return Y;
    }
};

template <typename R>
class Point<R,1> {
    R v;
public:
    static const unsigned size = 1;
    Point() {}
    template <typename R2>
    explicit Point(const Point<R2,1>& p) : v(R(p[0])) {}
    Point(R val) : v(std::move(val)) {};
    R& operator[](int i) { return v; }
    const R& operator[](int i) const { return v; }
    operator R() { return v; }
    typedef R value_type;
    template <typename R2> Point<R,1>& operator=( const Point<R2,1>& b) {
        v = R(b[0]);
        return *this;
    }
};

template <typename R>
class Point<R,2> {
public:
    static const unsigned size = 2;
    R x,y;
    Point() {}
    explicit Point(R val) : x(val), y(val) {};
    template <typename R2>
    explicit Point(const Point<R2,2>& p) : x(R(p.x)), y(R(p.y)) {}
    Point(R x, R y) : x(std::move(x)), y(std::move(y)) {};
    R& operator[](int i) { return ((R*)this)[i]; }
    const R& operator[](int i) const { return ((R*)this)[i]; }
    typedef R value_type;
    template <typename R2> Point<R,2>& operator=( const Point<R2,2>& b) {
        x = R(b.x); y = R(b.y);
        return *this;
    }
};

template <typename R>
class Point<R,3> {
public:
    static const unsigned size = 3;
    R x,y,z;
    Point() {}
    explicit Point(R val) : x(val), y(val), z(val) {};
    template <typename R2>
    explicit Point(const Point<R2,3>& p) {
        for( unsigned i=0; i<size; ++i) (*this)[i] = R(p[i]);
    }
    Point(R x, R y, R z) : x(std::move(x)),y(std::move(y)),z(std::move(z)) {};
    R& operator[](int i) { return ((R*)this)[i]; }
    const R& operator[](int i) const { return ((R*)this)[i]; }
    typedef R value_type;
    template <typename R2> Point<R,3>& operator=( const Point<R2,3>& b) {
        for( unsigned i=0; i < size; ++i) (*this)[i] = R(b[i]);
        return *this;
    }
};

template <typename R>
class Point<R,4> {
public:
    static const unsigned size = 4;
    R x,y,z,w;
    Point() {}
    explicit Point(R val) : x(val), y(val), z(val), w(val) {};
    template <typename R2>
    explicit Point(const Point<R2,4>& p) {
        for( unsigned i=0; i<size; ++i) (*this)[i] = R(p[i]);
    }
    Point(R x, R y, R z, R w) :
        x(std::move(x)),y(std::move(y)),z(std::move(z)),w(std::move(w)) {};
    R& operator[](int i) { return ((R*)this)[i]; }
    const R& operator[](int i) const { return ((R*)this)[i]; }
    typedef R value_type;
    template <typename R2> Point<R,4>& operator=( const Point<R2,4>& b) {
        for( unsigned i=0; i < size; ++i) (*this)[i] = R(b[i]);
        return *this;
    }
};

/**
 * This specialization is for RGB colours.
 */
template <typename R>
class Point<R,-1> {
public:
    static const unsigned size = 3;
    R r,g,b;
    Point() {}
    explicit Point(R val) : r(val), g(val), b(val) {};
    template <typename R2>
    explicit Point(const Point<R2,-1>& p) {
        for( unsigned i=0; i<size; ++i) (*this)[i] = R(p[i]);
    }
    template <typename R2>
    explicit Point(const Point<R2,-2>& p) : r(R(p.r)), g(R(p.g)), b(R(p.b)) {}
    Point(R r, R g, R b) : r(std::move(r)),g(std::move(g)),b(std::move(b)) {};
    R& operator[](int i) { return ((R*)this)[i]; }
    const R& operator[](int i) const { return ((R*)this)[i]; }
    typedef R value_type;
    template <typename R2> Point<R,-1>& operator=( const Point<R2,-1>& b) {
        for( unsigned i=0; i < size; ++i) (*this)[i] = R(b[i]);
        return *this;
    }
};

/**
 * This specialization is for BGR colours.
 */
template <typename R>
class Point<R,-2> {
public:
    static const unsigned size = 3;
    R b,g,r;
    Point() {}
    explicit Point(R val) : b(val), g(val), r(val) {};
    template <typename R2>
    explicit Point(const Point<R2,-2>& p) {
        for( unsigned i=0; i<size; ++i) (*this)[i] = R(p[i]);
    }
    template <typename R2>
    explicit Point(const Point<R2,-1>& p) : r(p.r), g(p.g), b(p.b) {}
    Point(R b, R g, R r) : b(std::move(b)),g(std::move(g)),r(std::move(r)) {};
    R& operator[](int i) { return ((R*)this)[i]; }
    const R& operator[](int i) const { return ((R*)this)[i]; }
    typedef R value_type;
    template <typename R2> Point<R,-2>& operator=( const Point<R2,-2>& b) {
        for( unsigned i=0; i < size; ++i) (*this)[i] = R(b[i]);
        return *this;
    }
};

typedef Point<double,2> point2d;
typedef Point<double,3> point3d;
typedef Point<unsigned char,-1> Color;
typedef Point<unsigned char,-1> RGBColor;
typedef Point<unsigned char,-2> BGRColor;

template <typename R,int n,typename R2>
inline bool operator ==( const Point<R,n>& a, const Point<R2,n>& b) {
    for( unsigned i=0; i < a.size; ++i)
        if( a[i] != b[i]) return 0;
    return 1;
}

template <typename R,int n,typename R2>
inline bool operator !=( const Point<R,n>& a, const Point<R2,n>& b) {
    return !(a == b);
}

template <typename R,int n,typename R2>
inline Point<R,n>& operator +=( Point<R,n>& a, const Point<R2,n>& b)
{
    for(unsigned i=0; i < a.size; ++i) a[i] += b[i];
    return a;
}

template <typename R,int n,typename R2>
inline Point<R,n> operator +( Point<R,n> a, const Point<R2,n>& b)
{
    return std::move(a += b);
}

template <typename R,int n>
inline Point<R,n> operator +( const Point<R,n>& a, Point<R,n>&& b)
{
    return std::move(b += a);
}

template <typename R,int n,typename R2>
inline Point<R,n>& operator -=( Point<R,n>& a, const Point<R2,n>& b)
{
    for(unsigned i=0; i < a.size; ++i) a[i] -= b[i];
    return a;
}

template <typename R,typename R2,int n>
inline Point<R,n> operator -( Point<R,n> a, const Point<R2,n>& b)
{
    return std::move(a -= b);
}

template <int n>
inline Point<int,n> operator -(
    const Point<unsigned char,n>& a, const Point<unsigned char,n>& b)
{
    Point<int,n> c;
    for(unsigned i=0; i < a.size; ++i) c[i] = a[i] - b[i];
    return c;
}

template <typename R,int n>
inline Point<R,n> operator -( Point<R,n> a)
{
    for(unsigned i=0; i < a.size; ++i) a[i] = -std::move(a[i]);
    return std::move(a);
}

template <typename R,int n,typename S>
inline Point<R,n>& operator *=( Point<R,n>& a, S b)
{
    for(unsigned i=0; i < a.size; ++i) a[i] *= b;
    return a;
}

template <typename R,int n>
inline Point<Point<R,n>,1> transpose( const Point<Point<R,1>,n>& a) {
    return *(Point<Point<R,n>,1>*)&a;
}

template <typename R,int n,typename S,
    typename std::enable_if<std::is_same<R,
        decltype(*(R*)0**(S*)0)>::value,int>::type=0>
inline Point<R,n> operator *( Point<R,n> a, S b)
{
    return std::move(a *= b);
}

template <typename R,int n,typename S,
    typename std::enable_if<std::is_same<R,
        decltype(*(R*)0**(S*)0)>::value,int>::type=0>
inline Point<R,n> operator *( S b, Point<R,n> a)
{
    return std::move(a *= b);
}

template <typename R,int n,typename S,
    typename std::enable_if<!std::is_same<R,
        decltype(*(R*)0**(S*)0)>::value,int>::type=0>
inline auto operator *( const Point<R,n>& a, S b) -> Point<decltype(a[0]*b),n>
{
    Point<decltype(a[0]*b),n> result;
    for(unsigned i=0; i < a.size; ++i) result[i] = a[i] * b;
    return std::move(result);
}

template <typename R,int n,typename S,
    typename std::enable_if<!std::is_same<R,
        decltype(*(R*)0**(S*)0)>::value,int>::type=0>
inline auto operator *( S b, const Point<R,n>& a) -> Point<decltype(a[0]*b),n>
{
    Point<decltype(a[0]*b),n> result;
    for(unsigned i=0; i < a.size; ++i) result[i] = a[i] * b;
    return std::move(result);
}

template <typename R,int n,typename S>
inline Point<R,n>& operator /=( Point<R,n>& a, S b)
{
    for(unsigned i=0; i < a.size; ++i) a[i] /= b;
    return a;
}

template <typename R,int n,typename S>
inline Point<R,n> operator /( Point<R,n> a, S b)
{
    return std::move(a /= b);
}

template <typename R,typename S,int n>
inline auto operator *( const Point<R,n>& a, const Point<S,n>& b)
    -> decltype(a[0]*b[0])
{
    auto c = a[0] * b[0];
    for(unsigned i=1; i < a.size; ++i) c += a[i] * b[i];
    return c;
}

template <typename R>
inline R operator *( const Point<R,0>& a, const Point<R,0>& b) { return R(0); }

// -- This operator is the CROSS-PRODUCT of "a" and "b"
template <typename R>
inline Point<R,3> operator /( const Point<R,3>& a, const Point<R,3>& b)
{
    Point<R,3> c (
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    ); // -- This is the CROSS-PRODUCT of "a" and "b"
    return c;
}

template <typename R,int n>
inline R sqlen( const Point<R,n>& a)
{
    return a * a;
}

template <typename R,int n>
inline Point<R,n> normalize(const Point<R,n>& a)
{
    return a / len(a);
}

template <typename R,int n>
inline R len( const Point<R,n>& a) { return sqrt(sqlen(a)); }

template <typename R,int n>
inline std::ostream& operator <<(std::ostream& os, const Point<R,n>& p)
{
    os << "[";
    for( unsigned i=0; i < p.size; ++i) {
        if( i > 0) os << ",";
        os << p[i];
    }
    return os << "]";
}

template <int n>
inline std::ostream& operator <<(std::ostream& os,
    const Point<unsigned char,n>& p)
{
    os << "[";
    for( unsigned i=0; i < p.size; ++i) {
        if( i > 0) os << ",";
        os << (int)p[i];
    }
    return os << "]";
}

inline std::ostream& operator <<(std::ostream& os,
    const Point<unsigned char,-1>& p)
{
    os << "#";
    for( int i=0; i < 3; ++i) {
        os << char((p[i] >> 4) + (p[i] < 160 ? '0' : 'a'-10));
        os << char((p[i] & 15) + ((p[i] & 15) < 10 ? '0' : 'a'-10));
    }
    return os;
}

template <typename R,int n>
inline std::ostream& operator <<(std::ostream& os,
    const SquareMatrix<R,n>& M)
{
    os << "[";
    for( int j=0; j < n; ++j) {
        for( int i=0; i < n; ++i) {
            os << M[j][i];
            if( i < n-1) os << " ";
        }
        if( j < n-1) os << ";";
    }
    return os << "]";
}

// Element-wise sqrt function
template <typename R,int n>
inline Point<R,n> sqrt( Point<R,n> a) {
    for( unsigned i=0; i < a.size; ++i) a[i] = sqrt(std::move(a[i]));
    return std::move(a);
}

// Element-wise multiplication function
template <typename R,int n>
inline Point<R,n> mult( Point<R,n> a, const Point<R,n>& b) {
    for( unsigned i=0; i < a.size; ++i) a[i] *= b[i];
    return std::move(a);
}
template <typename R,int n>
inline Point<R,n> mult( const Point<R,n>& a, Point<R,n>&& b) {
    return mult(std::move(b),a);
}

// Element-wise fabs function
template <typename R,int n>
inline Point<R,n> fabs( Point<R,n> a) {
    for( unsigned i=0; i < a.size; ++i) a[i] = fabs(std::move(a[i]));
    return std::move(a);
}

template <typename R>
inline Point<R,2> perp( Point<R,2> a) {
    R t = -std::move(a.y);
    a.y = std::move(a.x);
    a.x = std::move(t);
    return std::move(a);
}

template <typename ANGLE>
inline auto unitvector( ANGLE theta) -> Point<decltype(cos(theta)),2> {
    return Point<decltype(cos(theta)),2>( cos(theta), sin(theta));
}

template <typename R>
inline R angle( const Point<R,2>& vect) {
    return atan2( vect.y, vect.x);
}

template <typename R,typename ANGLE>
inline auto rotate(const Point<R,2>& a, ANGLE theta)->decltype(a*cos(theta)) {
    return a * cos(theta) + perp(a) * sin(theta);
}

template <typename R,int n,typename ANGLE>
inline Point<R,n> principalRotate( Point<R,n> a, ANGLE theta,
    unsigned d0, unsigned d1)
{
    auto c = cos(theta);
    auto s = sin(theta);
    auto t = a[d0] * c - a[d1] * s;
    a[d1] = a[d1] * c + a[d0] * s;
    a[d0] = t;
    return std::move(a);
}

// This function returns the image of point a after rotation by angle
// theta about the line through the origin pointing in the direction of
// unit vector u.  The rotation is right-handed in a right-handed
// coordinate system, or left-handed in a left-handed coordinate system.
// This function assumes that u has unit length.
template <typename R,typename ANGLE>
inline Point<R,3> rotateAboutAxis( const Point<R,3>& a,
    const Point<R,3>& u, ANGLE theta) {
    Point<R,3> z = u * (a * u);
    Point<R,3> x = a - z;
    Point<R,3> y = u / x;
    return z + x * cos(theta) + y * sin(theta);
}

// This is so we can get the expected value of non-gaussian variables.
template <typename R>
inline R E(R x) { return x; }

// This function returns the image of point a after rotation about the
// the line through the origin pointing in the direction of the given
// rotationVector.  The length of rotationVector is taken as the rotation
// angle in radians.
template <typename R>
inline Point<R,3> rotateAboutAxis( const Point<R,3>& a,
    const Point<R,3>& rotationVector) {
    auto theta = len(rotationVector);
    if( E(theta) == 0.0) return a;
    return rotateAboutAxis(a, rotationVector/theta, theta);
}

// This function is useful with Gaussian random variables.
template <typename R,int n>
inline auto E( const Point<R,n>& a) -> Point<decltype(E(a[0])),n> {
    Point<decltype(E(a[0])),n> result;
    for( unsigned i=0; i < a.size; ++i) result[i] = E(a[i]);
        return std::move(result);
}

template <typename R>
inline auto sinc(R x) -> decltype(sin(x)/x) {
    return E(x) == 0.0 ? 1.0 : sin(x)/x;
}

template <typename R>
inline Point<R,2>
eigenvalues_of_2x2_symmetric_matrix(const Point<R,3>& M)
{
    const auto a = (M[0] + M[2]) * 0.5;
    const auto b = sqrt(a*a - M[0]*M[2] + M[1]*M[1]);
    return Point<R,2>( a + b, a - b);
}

template <typename R>
inline Point<R,2>
eigenvector_of_2x2_symmetric_matrix( const Point<R,3>& M, R lambda)
{
    if(lambda == M[0]) return Point<R,2>( lambda - M[2], M[1]);
    return Point<R,2>( M[1], lambda - M[0]);
}

template <typename R>
inline Point<R,3>
eigenvalues_of_3x3_symmetric_matrix(const Point<R,6>& M)
{
    const double m = (M[0] + M[3] + M[5]) * (1.0/3.0);

    const double K11 = M[0] - m;
    const double K22 = M[3] - m;
    const double K33 = M[5] - m;

    const double M1212 = M[1] * M[1];
    const double M1313 = M[2] * M[2];
    const double M2323 = M[4] * M[4];

    const double q = M[1]*M[2]*M[4] + 0.5 *
        (K11*(K22*K33 - M2323) - M1212*K33 - M1313*K22);

    const double p =
        (1.0/3.0) * (M1212 + M1313 + M2323) +
        (1.0/6.0) * (K11*K11 + K22*K22 + K33*K33);

    const double phi = (1.0/3.0) * atan2(sqrt(p*p*p - q*q), q);
    const double sqrt_p = sqrt(p);
    const double x = sqrt_p * cos(phi);
    const double y = sqrt_p * sin(phi) * sqrt(3.0);

    return Point<R,3>( m+x+x, m-x-y, m-x+y);
}

template <typename R>
inline Point<R,3>
eigenvector_of_3x3_symmetric_matrix( const Point<R,6>& M, R lambda)
{
    Point<R,3> R1(M[0]-lambda, M[1], M[2]);
    Point<R,3> R2(M[1], M[3]-lambda, M[4]);
    Point<R,3> R3(M[2], M[4], M[5]-lambda);

    if( fabs(R3[0])>fabs(R2[0]) && fabs(R3[0])>fabs(R1[0])) std::swap(R1,R3);
    else if( fabs(R2[0])>fabs(R1[0])) std::swap(R1,R2);
    // Gaussian elimination of M-lambda*I
    if( R3[0] != 0) {
        const auto m = R3[0] / R1[0];
        R3[0] = 0;
        R3[1] -= R1[1] * m;
        R3[2] -= R1[2] * m;
    }
    if( R2[0] != 0) {
        const auto m = R2[0] / R1[0];
        R2[0] = 0;
        R2[1] -= R1[1] * m;
        R2[2] -= R1[2] * m;
    }
    if( fabs(R3[1]) > fabs(R2[1])) std::swap(R2,R3);
    if( R3[1] != 0) {
        R3[2] -= R2[2] * R3[1] / R2[1];
        R3[1] = 0;
    }

    // Now the matrix is upper-triangular.

    // We know that our matrix is theoretically singular, but rounding
    // errors may prevent it.  So let's enforce it by making the
    // smallest diagonal entry zero.  (We can do this because singular
    // triangular matrices always have a zero element on the diagonal.)
    // While we're at it, let's rearrange the matrix to make the bottom
    // row zero.
    if( fabs(R1[0]) < fabs(R2[1])) {
        if( fabs(R1[0]) < fabs(R3[2])) {
            R1[0] = 0;
            if( fabs(R2[1]) > fabs(R1[1])) std::swap(R1,R2);
            if( R2[1] != 0) {
                R2[2] -= R1[2] * R2[1] / R1[1];
                R2[1] = 0;
                if( fabs(R3[2]) > fabs(R2[2])) R2[2] = R3[2];
            }
        }
    } else {
        if( fabs(R2[1]) < fabs(R3[2])) {
            R2[1] = 0;
            if( fabs(R3[2]) > fabs(R2[2])) R2[2] = R3[2];
        }
    }
    R3[2] = 0;

    // Now finish simplifying the matrix
    if( R2[1] != 0) {
        if( R1[0] != 0) {
            R1[2] -= R2[2] * R1[1] / R2[1];
            R1[1] = 0;
        } else {
            if( fabs(R2[1]) > fabs(R1[1])) std::swap(R1,R2);
            if( R2[1] != 0) {
                R2[2] -= R1[2] * R2[1] / R1[1];
                R2[1] = 0;
            }
            if( R2[2] != 0) R1[2] = 0;
        }
    } else if( R2[2] != 0) R1[2] = 0;

    if( R2[1] != 0) return Point<R,3>(-R1[2]/R1[0],-R2[2]/R2[1],1);
    if( R1[0] != 0) return Point<R,3>(-R1[1]/R1[0],1,0);
    return Point<R,3>(1,0,0);
}

template <typename RowSwappableMatrix,typename Vector>
inline void gauss_jordon_partial_pivot(
    RowSwappableMatrix A, Vector b, Vector x, int N)
{
    // Triangularize
    for( int n=N-1; n > 0; --n) {
        // Select the pivot value
        int pivotRow = 0;
        auto pivotVal = A[0][n];
        for( int j=1; j <= n; ++j) {
            auto val = A[j][n];
            if( fabs(E(val)) > fabs(E(pivotVal))) {
                pivotRow = j;
                pivotVal = val;
            }
        }

        // Move the pivot to the bottom row
        if(pivotRow != n) {
            std::swap(A[n],A[pivotRow]);
            std::swap(b[n],b[pivotRow]);
        }

        // Eliminate the rest of the column
        for( int j=0; j < n; ++j) {
            auto elim = A[j][n] / pivotVal;
            for( int i=0; i < n; ++i)
                A[j][i] -= A[n][i] * elim;
            b[j] -= b[n] * elim;
        }
    }

    // Solve
    for( int n=0; n < N; ++n) {
        auto val = b[n];
        for( int i=0; i < n; ++i)
            val -= A[n][i] * x[i];
        x[n] = val / A[n][n];
    }
}

template <typename R,typename S,int n>
inline Point<R,n> solve_system_of_linear_equations(
    SquareMatrix<S,n>& A, Point<R,n>& b)
{
    Point<R,n> x;
    gauss_jordon_partial_pivot( A.getSwappableRows(), &b[0], &x[0], n);
    return x;
}

template <typename R,int n>
void invert_matrix( SquareMatrix<R,n>& M)
{
    Point<Point<R,n>,n> I,X;
    for( int j=0; j < n; ++j) {
        I[j][j] = 1.0;
        for( int i=0; i < j; ++i) I[j][i] = I[i][j] = 0.0;
    }
    gauss_jordon_partial_pivot( M.getSwappableRows(), &I[0], &X[0], n);
    for( int j=0; j < n; ++j)
        for( int i=0; i < n; ++i) M[j][i] = X[j][i];
}

template <typename R,int n>
inline void solve_lower_triangular_system_of_linear_equations(
    const SquareMatrix<R,n>& A, Point<R,n>& b)
{
    for( int j=0; j < n; ++j) {
        for( int i=0; i < j; ++i) b[j] -= A[j][i] * b[i];
        b[j] /= A[j][j];
    }
}

template <typename R,int n>
inline Point<R,n> ordinary_least_squares(
    const Point<R,n>* x, const R* y, int N)
{
    SquareMatrix<R,n> A;
    for( int j=0; j < n; ++j) {
        for( int i=j; i < n; ++i) {
            auto val = x[0][j] * x[0][i];
            for( int k=1; k < N; ++k)
                val += x[k][j] * x[k][i];
            A[j][i] = val;
            if(j != i) A[i][j] = val;
        }
    }
    Point<R,n> b = x[0] * y[0];
    for( int k=1; k < N; ++k)
        b += x[k] * y[k];

    return solve_system_of_linear_equations(A,b);
}

/**
 * @param model0 The initial guess of the model parameters.
 */
template <typename R,int n,typename S,typename T,typename MODEL_FUNCTION>
inline Point<R,n> nonlinear_least_squares(
    Point<R,n> model0, MODEL_FUNCTION f, const S* x, const T* y, int N)
{
    Point<R,n> model = model0;
    std::vector<decltype(len(y[0]-y[0]))> residuals(N);
    std::vector<Point<decltype((len(y[0]-y[0])-len(y[0]-y[0]))/(model0[0]-model0[0])),n>>
        jacobian(N);

    R alpha = 1e-4;

    for(int iter=0; iter < 500; ++iter) {
        // Compute the residuals
        for( int j=0; j < N; ++j)
            residuals[j] = sqlen(y[j] - f(x[j], model));

        // Compute the Jacobian
        for( int j=0; j < N; ++j) {
            Point<R,n> model_p = model;

            for( int i=0; i < n; ++i) {
                // Perturb the parameter
                if( fabs(E(model[i])) < 1e-8)
                    model_p[i] = 1e-8;
                else
                    model_p[i] *= 1.0000001;

                // Compute the perturbed residual
                auto r = sqlen(y[j] - f(x[j],model_p));

                jacobian[j][i] = (r - residuals[j]) /
                    (model_p[i] - model[i]);

                // Restore the perturbed parameter
                model_p[i] = model[i];
            }
        }

#if 1  // perform gradient descent
        Point<R,n> gradient(0.0);
        R error0 = 0;
        for (int i = 0; i < N; i++) {
            gradient += jacobian[i];
            error0 += residuals[i];
        }

        if (len(gradient) < 0.001) {
            std::cout << "converged! " << iter << "\n";
            break;
        }

        R error1 = 0;
        /* R alphak = alpha; */
        const int n_tries = 30;
        int k;
        for (k = 0; k < n_tries; k++) {
            error1 = 0;
            for(int i = 0; i < N; i++)
                error1 += sqlen(y[i] - f(x[i], model - alpha * gradient));

            if (error1 < error0) {
                break;
            } else
                alpha /= 2.0;
        }
        if (k == n_tries)
            throw 0;

        model -= alpha * gradient;
        alpha *= 2.0;
#else  // Gauss-Newton algorithm
        // Solve for the improved model parameters
        model -= ordinary_least_squares( &jacobian[0], &residuals[0], N);
#endif
    }

    return model;
}

template <typename R,int n>
inline void cholesky_decomposition(SquareMatrix<R,n>& A)
{
    for( int j=0; j < n; ++j) {
        R sum = 0.0;
        for( int k=0; k < j; ++k)
            sum += A[j][k] * A[j][k];
        A[j][j] = sqrt(A[j][j] - sum);
        double s = 1.0 / A[j][j];
        for( int i=j+1; i < n; ++i) {
            R sum = 0.0;
            for( int k=0; k < j; ++k)
                sum += A[i][k] * A[j][k];
            A[i][j] = (A[i][j] - sum) * s;
            A[j][i] = 0.0;
        }
    }
}

template <typename R,int n>
void invert_lower_triangular_matrix(SquareMatrix<R,n>& A)
{
    for( int j=0; j < n; ++j) {
        R r = R(1.0) / A[j][j];
        for( int i=0; i < j; ++i) {
            R x = -A[i][i] * A[j][i];
            for( int k=i+1; k < j; ++k)
                x -= A[k][i] * A[j][k];
            A[j][i] = x * r;
        }
        A[j][j] = r;
    }
}

template <typename R,int n>
void invert_symmetric_positive_definite_matrix(SquareMatrix<R,n>& A)
{
    cholesky_decomposition(A);
    invert_lower_triangular_matrix(A);
    for( int j=0; j < n; ++j) {
        for( int i=0; i <= j; ++i) {
            A[j][i] *= A[j][j];
            for( int k=j+1; k < n; ++k)
                A[j][i] += A[k][i] * A[k][j];
            A[i][j] = A[j][i];
        }
    }
}

template <typename T>
class HyperCube {
    HyperCube(int*) {}  // Dummy constructor for factory functions

    template <typename S>
    static S dropref(const S&) { return *(S*)0; }

    static decltype(dropref((*(T*)0)[0])) inf() {
        return std::numeric_limits<decltype(dropref((*(T*)0)[0]))>::infinity();
    }

public:
    T min, max;

    // Construct an empty hypercube
    HyperCube() : min(inf()), max(-inf()) {}

    HyperCube( const HyperCube& a) : min(a.min), max(a.max) {}

    HyperCube( const T& min, const T& max) :
        min(min), max(max) {}

    bool contains( const T& p) const {
        for( unsigned i=0; i < T::size; ++i)
            if( p[i] < min[i] || p[i] > max[i]) return false;
        return true;
    }

    bool contains( const HyperCube& b) const {
        for( unsigned i=0; i < T::size; ++i)
            if( b.min[i] < min[i] || b.max[i] > max[i]) return false;
        return true;
    }

    void insert( const T& p) {
        for( unsigned i=0; i < T::size; ++i) {
            if( p[i] < min[i]) min[i] = p[i];
            if( p[i] > max[i]) max[i] = p[i];
        }
    }

    void insert( const HyperCube& a)
    {
        for( unsigned i=0; i < T::size; ++i) {
            if( a.min[i] < min[i]) min[i] = a.min[i];
            if( a.max[i] > max[i]) max[i] = a.max[i];
        }
    }

    bool intersects( const HyperCube& b) const
    {
        for( unsigned i=0; i < T::size; ++i)
            if( b.min[i] > max[i] || b.max[i] < min[i]) return false;
        return true;
    }

    void intersect( const HyperCube& b)
    {
        for( unsigned i=0; i < T::size; ++i) {
            min[i] = std::max(min[i], b.min[i]);
            max[i] = std::min(max[i], b.max[i]);
            if( min[i] > max[i]) {
                min = T(inf());
                max = T(-inf());
                return;
            }
        }
    }

    friend HyperCube intersect( const HyperCube& a, const HyperCube& b)
    {
        HyperCube c((int*)0);
        for( unsigned i=0; i < T::size; ++i) {
            c.min[i] = std::max(a.min[i], b.min[i]);
            c.max[i] = std::min(a.max[i], b.max[i]);
            if( c.min[i] > c.max[i]) return HyperCube();
        }
        return c;
    }

    friend HyperCube unite( const HyperCube& a, const HyperCube& b)
    {
        HyperCube c((int*)0);
        for( unsigned i=0; i < T::size; ++i) {
            c.min[i] = std::min(a.min[i], b.min[i]);
            c.max[i] = std::max(a.max[i], b.max[i]);
        }
        return c;
    }
};

typedef HyperCube<Point<double,1>> range;
typedef HyperCube<Point<double,2>> bbox2d;
typedef HyperCube<Point<double,3>> bbox3d;

#endif
