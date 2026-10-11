using System;

namespace QuatLab
{
    /// <summary>轻量三维向量（字段 X/Y/Z 与 UnityEngine.Vector3 对齐，方便互转）。</summary>
    public struct Vec3
    {
        public float X, Y, Z;

        public Vec3(float x, float y, float z)
        {
            X = x; Y = y; Z = z;
        }

        public static readonly Vec3 Zero = new Vec3(0f, 0f, 0f);

        public float LengthSquared
        {
            get { return X * X + Y * Y + Z * Z; }
        }

        public float Length
        {
            get { return (float)Math.Sqrt(LengthSquared); }
        }

        public bool IsZero
        {
            get { return LengthSquared < 1e-12f; }
        }

        public Vec3 Normalized()
        {
            float l = Length;
            if (l < 1e-12f)
                return Zero;
            float inv = 1f / l;
            return new Vec3(X * inv, Y * inv, Z * inv);
        }

        public static Vec3 operator +(Vec3 a, Vec3 b)
        {
            return new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Vec3 operator -(Vec3 a, Vec3 b)
        {
            return new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Vec3 operator -(Vec3 a)
        {
            return new Vec3(-a.X, -a.Y, -a.Z);
        }

        public static Vec3 operator *(Vec3 a, float s)
        {
            return new Vec3(a.X * s, a.Y * s, a.Z * s);
        }

        public static Vec3 operator *(float s, Vec3 a)
        {
            return new Vec3(a.X * s, a.Y * s, a.Z * s);
        }

        public static float Dot(Vec3 a, Vec3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        }

        public override string ToString()
        {
            return "(" + Fmt(X) + ", " + Fmt(Y) + ", " + Fmt(Z) + ")";
        }

        private static string Fmt(float v)
        {
            return v.ToString("F3");
        }
    }
}
