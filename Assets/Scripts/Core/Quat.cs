using System;

namespace QuatLab
{
    /// <summary>
    /// 四元数 (W, X, Y, Z)。
    /// 约定：对向量 v 的旋转为 q v q⁻¹；两个旋转的复合为 Hamilton 积。
    /// </summary>
    public struct Quat
    {
        public float W, X, Y, Z;

        public Quat(float w, float x, float y, float z)
        {
            W = w; X = x; Y = y; Z = z;
        }

        public static readonly Quat Identity = new Quat(1f, 0f, 0f, 0f);

        public float LengthSquared
        {
            get { return W * W + X * X + Y * Y + Z * Z; }
        }

        public float Length
        {
            get { return (float)Math.Sqrt(LengthSquared); }
        }

        public Quat Normalized()
        {
            float l = Length;
            if (l < 1e-12f)
                return Identity;
            float inv = 1f / l;
            return new Quat(W * inv, X * inv, Y * inv, Z * inv);
        }

        public Quat Conjugate()
        {
            return new Quat(W, -X, -Y, -Z);
        }

        public Quat Inverse()
        {
            float l2 = LengthSquared;
            if (l2 < 1e-12f)
                return Identity;
            return new Quat(W / l2, -X / l2, -Y / l2, -Z / l2);
        }

        public static Quat operator +(Quat a, Quat b)
        {
            return new Quat(a.W + b.W, a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Quat operator -(Quat a)
        {
            return new Quat(-a.W, -a.X, -a.Y, -a.Z);
        }

        public static Quat operator -(Quat a, Quat b)
        {
            return new Quat(a.W - b.W, a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Quat operator *(Quat a, float s)
        {
            return new Quat(a.W * s, a.X * s, a.Y * s, a.Z * s);
        }

        public static Quat operator *(float s, Quat a)
        {
            return new Quat(a.W * s, a.X * s, a.Y * s, a.Z * s);
        }

        /// <summary>Hamilton 积：先 a 后 b 的复合旋转为 b * a。</summary>
        public static Quat operator *(Quat a, Quat b)
        {
            return new Quat(
                a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
                a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
                a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
                a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W);
        }

        /// <summary>用四元数旋转向量：q v q⁻¹。</summary>
        public static Vec3 Rotate(Quat q, Vec3 v)
        {
            // t = 2 * cross(q.xyz, v);  v' = v + w*t + cross(q.xyz, t)
            Vec3 u = new Vec3(q.X, q.Y, q.Z);
            Vec3 t = Vec3.Cross(u, v) * 2f;
            return v + t * q.W + Vec3.Cross(u, t);
        }

        public static float Dot(Quat a, Quat b)
        {
            return a.W * b.W + a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public override string ToString()
        {
            return "(" + Fmt(W) + ", " + Fmt(X) + ", " + Fmt(Y) + ", " + Fmt(Z) + ")";
        }

        private static string Fmt(float v)
        {
            return v.ToString("F3");
        }
    }
}
