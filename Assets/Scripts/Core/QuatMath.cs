using System;
using System.Collections.Generic;

namespace QuatLab
{
    /// <summary>3x3 旋转矩阵（行主序，作用：v' = M v）。</summary>
    public struct Matrix3
    {
        public float M00, M01, M02;
        public float M10, M11, M12;
        public float M20, M21, M22;

        public Matrix3(float m00, float m01, float m02,
                       float m10, float m11, float m12,
                       float m20, float m21, float m22)
        {
            M00 = m00; M01 = m01; M02 = m02;
            M10 = m10; M11 = m11; M12 = m12;
            M20 = m20; M21 = m21; M22 = m22;
        }

        public static readonly Matrix3 Identity =
            new Matrix3(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f);

        public Vec3 MultiplyVec(Vec3 v)
        {
            return new Vec3(
                M00 * v.X + M01 * v.Y + M02 * v.Z,
                M10 * v.X + M11 * v.Y + M12 * v.Z,
                M20 * v.X + M21 * v.Y + M22 * v.Z);
        }

        public static bool IsIdentity(Matrix3 m, float eps)
        {
            return Math.Abs(m.M00 - 1f) < eps && Math.Abs(m.M11 - 1f) < eps && Math.Abs(m.M22 - 1f) < eps
                && Math.Abs(m.M01) < eps && Math.Abs(m.M02) < eps
                && Math.Abs(m.M10) < eps && Math.Abs(m.M12) < eps
                && Math.Abs(m.M20) < eps && Math.Abs(m.M21) < eps;
        }

        public float Determinant()
        {
            return M00 * (M11 * M22 - M12 * M21)
                 - M01 * (M10 * M22 - M12 * M20)
                 + M02 * (M10 * M21 - M11 * M20);
        }

        public override string ToString()
        {
            return "[" + Fmt(M00) + " " + Fmt(M01) + " " + Fmt(M02) + "]\n"
                 + "[" + Fmt(M10) + " " + Fmt(M11) + " " + Fmt(M12) + "]\n"
                 + "[" + Fmt(M20) + " " + Fmt(M21) + " " + Fmt(M22) + "]";
        }

        private static string Fmt(float v)
        {
            return v.ToString("F3");
        }
    }

    /// <summary>四元数构造/转换/插值等工具函数（全部为静态纯函数，可离线测试）。</summary>
    public static class QuatMath
    {
        private const float Rad2Deg = 57.29577951308232f;
        private const float Deg2Rad = 0.017453292519943295f;

        private static float Clamp(float v, float lo, float hi)
        {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }

        private static float AcosClamped(float v)
        {
            return (float)Math.Acos(Clamp(v, -1f, 1f));
        }

        /// <summary>由轴-角构造四元数（右手系，角度单位：度）。</summary>
        public static Quat FromAxisAngle(Vec3 axis, float angleDeg)
        {
            if (axis.IsZero)
                return Quat.Identity;
            Vec3 n = axis.Normalized();
            float half = angleDeg * Deg2Rad * 0.5f;
            float s = (float)Math.Sin(half);
            return new Quat((float)Math.Cos(half), n.X * s, n.Y * s, n.Z * s);
        }

        /// <summary>四元数分解为轴-角（角度单位：度）。</summary>
        public static void ToAxisAngle(Quat q, out Vec3 axis, out float angleDeg)
        {
            Quat n = q.Normalized();
            float w = Clamp(n.W, -1f, 1f);
            float half = AcosClamped(w);
            float s = (float)Math.Sin(half);
            angleDeg = half * 2f * Rad2Deg;
            if (s < 1e-6f)
            {
                axis = new Vec3(1f, 0f, 0f);
                return;
            }
            axis = new Vec3(n.X / s, n.Y / s, n.Z / s);
        }

        /// <summary>
        /// 由欧拉角构造四元数。约定：先绕 Z 转 roll，再绕 X 转 pitch，最后绕 Y 转 yaw（R = Ry * Rx * Rz）。
        /// 与 Unity Quaternion.Euler(x, y, z) 的应用顺序一致（Z→X→Y）。
        /// </summary>
        public static Quat FromEulerYXZ(float yawDeg, float pitchDeg, float rollDeg)
        {
            Quat ry = FromAxisAngle(new Vec3(0f, 1f, 0f), yawDeg);
            Quat rx = FromAxisAngle(new Vec3(1f, 0f, 0f), pitchDeg);
            Quat rz = FromAxisAngle(new Vec3(0f, 0f, 1f), rollDeg);
            return ry * rx * rz;
        }

        /// <summary>四元数分解为欧拉角（度），与 FromEulerYXZ 互逆。</summary>
        public static void ToEulerYXZ(Quat q, out float yawDeg, out float pitchDeg, out float rollDeg)
        {
            Matrix3 m = ToMatrix3(q.Normalized());

            // R = Ry*Rx*Rz，R[1][2] = -sin(pitch)
            float sx = -m.M12;
            sx = Clamp(sx, -1f, 1f);
            pitchDeg = (float)Math.Asin(sx) * Rad2Deg;

            if (Math.Abs(sx) < 0.99999f)
            {
                yawDeg = (float)Math.Atan2(m.M02, m.M22) * Rad2Deg;
                rollDeg = (float)Math.Atan2(m.M10, m.M11) * Rad2Deg;
            }
            else
            {
                // 万向锁（pitch = ±90°）：取 roll = 0
                yawDeg = (float)Math.Atan2(-m.M20, m.M00) * Rad2Deg;
                rollDeg = 0f;
            }
        }

        /// <summary>四元数 → 旋转矩阵（行主序，作用于列向量）。</summary>
        public static Matrix3 ToMatrix3(Quat q)
        {
            float x = q.X, y = q.Y, z = q.Z, w = q.W;
            float xx = x * x, yy = y * y, zz = z * z;
            float xy = x * y, xz = x * z, yz = y * z;
            float wx = w * x, wy = w * y, wz = w * z;

            return new Matrix3(
                1f - 2f * (yy + zz), 2f * (xy - wz), 2f * (xz + wy),
                2f * (xy + wz), 1f - 2f * (xx + zz), 2f * (yz - wx),
                2f * (xz - wy), 2f * (yz + wx), 1f - 2f * (xx + yy));
        }

        /// <summary>旋转矩阵 → 四元数（Shepperd 分支法，数值稳定）。</summary>
        public static Quat FromMatrix3(Matrix3 m)
        {
            float tr = m.M00 + m.M11 + m.M22;
            if (tr > 0f)
            {
                float s = (float)Math.Sqrt(tr + 1f) * 2f;
                return new Quat(0.25f * s,
                    (m.M21 - m.M12) / s,
                    (m.M02 - m.M20) / s,
                    (m.M10 - m.M01) / s).Normalized();
            }
            if (m.M00 > m.M11 && m.M00 > m.M22)
            {
                float s = (float)Math.Sqrt(1f + m.M00 - m.M11 - m.M22) * 2f;
                return new Quat(
                    (m.M21 - m.M12) / s,
                    0.25f * s,
                    (m.M01 + m.M10) / s,
                    (m.M02 + m.M20) / s).Normalized();
            }
            if (m.M11 > m.M22)
            {
                float s = (float)Math.Sqrt(1f + m.M11 - m.M00 - m.M22) * 2f;
                return new Quat(
                    (m.M02 - m.M20) / s,
                    (m.M01 + m.M10) / s,
                    0.25f * s,
                    (m.M12 + m.M21) / s).Normalized();
            }
            float s2 = (float)Math.Sqrt(1f + m.M22 - m.M00 - m.M11) * 2f;
            return new Quat(
                (m.M10 - m.M01) / s2,
                (m.M02 + m.M20) / s2,
                (m.M12 + m.M21) / s2,
                0.25f * s2).Normalized();
        }

        /// <summary>两个四元数间的夹角（度）。</summary>
        public static float AngleBetween(Quat a, Quat b)
        {
            float d = Clamp(Math.Abs(Quat.Dot(a.Normalized(), b.Normalized())), -1f, 1f);
            return AcosClamped(d) * 2f * Rad2Deg;
        }

        /// <summary>球面线性插值（单位四元数，沿最短弧匀速转动）。</summary>
        public static Quat Slerp(Quat a, Quat b, float t)
        {
            float dot = Clamp(Quat.Dot(a, b), -1f, 1f);
            if (dot < 0f)
            {
                b = -b;
                dot = -dot;
            }
            if (dot > 0.9995f)
                return Lerp(a, b, t).Normalized();

            float theta = AcosClamped(dot);
            float sinTheta = (float)Math.Sin(theta);
            float w0 = (float)Math.Sin((1f - t) * theta) / sinTheta;
            float w1 = (float)Math.Sin(t * theta) / sinTheta;
            return a * w0 + b * w1;
        }

        /// <summary>线性插值（结果不一定为单位长度，速度不均匀）。</summary>
        public static Quat Lerp(Quat a, Quat b, float t)
        {
            return a + (b - a) * t;
        }

        /// <summary>沿 Slerp 路径采样 steps+1 个四元数（动画轨迹用）。</summary>
        public static List<Quat> SlerpPath(Quat a, Quat b, int steps)
        {
            List<Quat> path = new List<Quat>(steps + 1);
            for (int i = 0; i <= steps; i++)
                path.Add(Slerp(a, b, (float)i / steps));
            return path;
        }
    }
}
