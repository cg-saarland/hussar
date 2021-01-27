RT_FUNCTION float3 vec3_to_float3(const Vector3f &vec) {
    return float3 { vec.x(), vec.y(), vec.z() };
}

RT_FUNCTION Vector3f float3_to_vec3(const float3 &f) {
    return Vector3f(f.x, f.y, f.z);
}

RT_FUNCTION Intersection &getIsect() {
    return *reinterpret_cast<Intersection *>(unpackPointer(optixGetPayload_0(), optixGetPayload_1()));
}

RT_FUNCTION float3 make_float3(const float4 &f) {
    return { f.x, f.y, f.z };
}

RT_FUNCTION float dot(const float3 &a, const float3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

RT_FUNCTION float3 cross(const float3 &a, const float3 &b) {
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

RT_FUNCTION float length(const float3 &v) {
    return sqrtf(dot(v, v));
}

RT_FUNCTION float3 operator*(const float3 &a, const float3 &b) {
    return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

RT_FUNCTION float3 operator*(const float3 &a, const float s) {
    return make_float3(a.x * s, a.y * s, a.z * s);
}

RT_FUNCTION float3 operator*(const float s, const float3 &a) {
    return make_float3(a.x * s, a.y * s, a.z * s);
}

RT_FUNCTION float3 operator-(const float3 &a, const float3 &b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

RT_FUNCTION float3 operator+(const float3 &a, const float3 &b) {
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

RT_FUNCTION float3 operator-(const float3 &a) {
    return make_float3(-a.x, -a.y, -a.z);
}

RT_FUNCTION float3 normalize(const float3 &v) {
  float invLen = 1.0f / sqrtf(dot(v, v));
    return v * invLen;
}

RT_FUNCTION float3 faceforward(const float3 &n, const float3 &i, const float3 &nref) {
    return n * copysignf(1.0f, dot(i, nref));
}
