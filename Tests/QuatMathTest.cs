using System;
using System.Collections.Generic;
using QuatLab;

/// <summary>四元数数学核心离线测试（csc 编译运行，ALL PASS 退出码 0）。</summary>
public static class QuatMathTest
{
    private static int passCount, failCount;

    private static void Check(bool cond, string name)
    {
        if (cond)
        {
            passCount++;
            Console.WriteLine("   PASS: " + name);
        }
        else
        {
            failCount++;
            Console.WriteLine("   FAIL: " + name);
        }
    }

    private static bool Near(float a, float b, float eps)
    {
        return Math.Abs(a - b) <= eps;
    }

    private static bool NearVec(Vec3 a, Vec3 b, float eps)
    {
        return Near(a.X, b.X, eps) && Near(a.Y, b.Y, eps) && Near(a.Z, b.Z, eps);
    }

    /// <summary>四元数相等判定（考虑 ±q 等价）。</summary>
    private static bool NearQuat(Quat a, Quat b, float eps)
    {
        float d = Math.Abs(Quat.Dot(a.Normalized(), b.Normalized()));
        return d >= 1f - eps;
    }

    private static bool SameRotation(Quat a, Quat b, float eps)
    {
        return NearQuat(a, b, eps) || NearQuat(a, -b, eps);
    }

    private static float Deg(float rad)
    {
        return rad * 57.29577951308232f;
    }

    private static void Main()
    {
        Console.WriteLine("== 四元数旋转变换核心测试 ==");
        float EPS = 1e-4f;

        // ---- 基础旋转（90° 绕轴，右手系）----
        Quat qx90 = QuatMath.FromAxisAngle(new Vec3(1, 0, 0), 90f);
        Quat qy90 = QuatMath.FromAxisAngle(new Vec3(0, 1, 0), 90f);
        Quat qz90 = QuatMath.FromAxisAngle(new Vec3(0, 0, 1), 90f);

        Check(NearVec(Quat.Rotate(qx90, new Vec3(0, 1, 0)), new Vec3(0, 0, 1), EPS), "Rx90: (0,1,0)->(0,0,1)");
        Check(NearVec(Quat.Rotate(qx90, new Vec3(0, 0, 1)), new Vec3(0, -1, 0), EPS), "Rx90: (0,0,1)->(0,-1,0)");
        Check(NearVec(Quat.Rotate(qy90, new Vec3(1, 0, 0)), new Vec3(0, 0, -1), EPS), "Ry90: (1,0,0)->(0,0,-1)");
        Check(NearVec(Quat.Rotate(qz90, new Vec3(1, 0, 0)), new Vec3(0, 1, 0), EPS), "Rz90: (1,0,0)->(0,1,0)");
        Check(NearVec(Quat.Rotate(qy90, new Vec3(0, 0, 1)), new Vec3(1, 0, 0), EPS), "Ry90: (0,0,1)->(1,0,0)");

        // 180° 绕 Y：翻转 X 与 Z
        Quat qy180 = QuatMath.FromAxisAngle(new Vec3(0, 1, 0), 180f);
        Check(NearVec(Quat.Rotate(qy180, new Vec3(1, 0, 0)), new Vec3(-1, 0, 0), EPS), "Ry180: X 翻转");
        Check(NearVec(Quat.Rotate(qy180, new Vec3(0, 0, 1)), new Vec3(0, 0, -1), EPS), "Ry180: Z 翻转");

        // ---- 单位性与逆 ----
        Check(Near(Quat.Rotate(Quat.Identity, new Vec3(3, -2, 5)).X, 3f, EPS) &&
              Near(Quat.Rotate(Quat.Identity, new Vec3(3, -2, 5)).Y, -2f, EPS) &&
              Near(Quat.Rotate(Quat.Identity, new Vec3(3, -2, 5)).Z, 5f, EPS), "恒等旋转保持向量不变");
        Check(Near(qz90.Normalized().Length, 1f, 1e-5f), "四元数归一化后长度为 1");
        Check(SameRotation(qx90 * qx90.Conjugate(), Quat.Identity, EPS), "q*conj(q) ≈ 恒等");
        Check(SameRotation(qx90.Inverse() * qx90, Quat.Identity, EPS), "q⁻¹*q ≈ 恒等");
        Check(NearVec(Quat.Rotate(qx90.Inverse(), Quat.Rotate(qx90, new Vec3(1, 2, 3))), new Vec3(1, 2, 3), EPS), "逆旋转还原向量");
        Check(SameRotation(qz90 * Quat.Identity, qz90, EPS), "右乘恒等不变");

        // ---- 旋转保长度/保内积 ----
        Vec3 u = new Vec3(1, 2, 2);
        Vec3 v = new Vec3(-2, 1, 3);
        Check(Near(Quat.Rotate(qz90, u).Length, u.Length, EPS), "旋转保长度");
        Check(Near(Vec3.Dot(Quat.Rotate(qz90, u), Quat.Rotate(qz90, v)), Vec3.Dot(u, v), EPS), "旋转保内积");

        // ---- 复合与非交换 ----
        Check(NearVec(Quat.Rotate(qz90, Quat.Rotate(qy90, new Vec3(1, 0, 0))),
                      Quat.Rotate(qz90 * qy90, new Vec3(1, 0, 0)), EPS), "复合：先 a 后 b 等于 (b*a) 旋转");
        Vec3 testV = new Vec3(0.7f, -1.1f, 0.4f);
        bool nonComm = !NearVec(Quat.Rotate(qx90 * qy90, testV), Quat.Rotate(qy90 * qx90, testV), 1e-3f);
        Check(nonComm, "四元数乘法不可交换（Rx*Ry ≠ Ry*Rx）");

        // ---- 轴-角互转 ----
        Quat qArb = QuatMath.FromAxisAngle(new Vec3(1, -2, 3), 123f);
        Vec3 axisBack;
        float angleBack;
        QuatMath.ToAxisAngle(qArb, out axisBack, out angleBack);
        Check(Near(angleBack, 123f, 0.2f), "轴-角互转：角度还原 (123°)");
        Check(SameRotation(QuatMath.FromAxisAngle(axisBack, angleBack), qArb, EPS), "轴-角互转：四元数还原");
        float angId;
        QuatMath.ToAxisAngle(Quat.Identity, out axisBack, out angId);
        Check(Near(angId, 0f, 1e-3f), "恒等四元数轴-角角度为 0");
        Vec3 axZ;
        float angZ;
        QuatMath.ToAxisAngle(qz90, out axZ, out angZ);
        Check(Near(angZ, 90f, 0.2f) &&
              (NearVec(axZ, new Vec3(0, 0, 1), EPS) || NearVec(axZ, new Vec3(0, 0, -1), EPS)), "Rz90 轴为 ±Z、角 90°");

        // ---- 轴-角边界 ----
        Check(SameRotation(QuatMath.FromAxisAngle(Vec3.Zero, 45f), Quat.Identity, EPS), "零轴返回恒等（防除零）");
        Check(SameRotation(QuatMath.FromAxisAngle(new Vec3(0, 1, 0), 0f), Quat.Identity, EPS), "角度 0 为恒等");
        Check(SameRotation(QuatMath.FromAxisAngle(new Vec3(0, 1, 0), 360f), Quat.Identity, EPS), "角度 360 为恒等");

        // ---- 欧拉角（约定 R = Ry*Rx*Rz）----
        Check(SameRotation(QuatMath.FromEulerYXZ(0f, 0f, 0f), Quat.Identity, EPS), "欧拉(0,0,0) 为恒等");
        Check(NearVec(Quat.Rotate(QuatMath.FromEulerYXZ(0f, 90f, 0f), new Vec3(0, 1, 0)), new Vec3(0, 0, 1), EPS),
              "欧拉 pitch=90° 等价 Rx90");
        Check(NearVec(Quat.Rotate(QuatMath.FromEulerYXZ(90f, 0f, 0f), new Vec3(1, 0, 0)), new Vec3(0, 0, -1), EPS),
              "欧拉 yaw=90° 等价 Ry90");

        float y0, p0, r0;
        QuatMath.ToEulerYXZ(qArb, out y0, out p0, out r0);
        Check(SameRotation(QuatMath.FromEulerYXZ(y0, p0, r0), qArb, EPS), "欧拉互转：往返还原");
        float y1, p1, r1;
        QuatMath.ToEulerYXZ(qx90, out y1, out p1, out r1);
        Check(Near(p1, 90f, 0.2f), "Rx90 的 pitch ≈ 90°");

        // 万向锁：pitch=±90 时 yaw+roll 合并
        // pitch=90 时合并角为 yaw-roll：30-45 = -15
        Quat g1 = QuatMath.FromEulerYXZ(30f, 90f, 45f);
        Quat g2 = QuatMath.FromEulerYXZ(-15f, 90f, 0f);
        Check(SameRotation(g1, g2, 1e-3f), "万向锁：pitch=90 时合并角为 yaw-roll");

        // ---- 矩阵互转 ----
        Check(Matrix3.IsIdentity(QuatMath.ToMatrix3(Quat.Identity), EPS), "恒等四元数矩阵为恒等阵");
        Matrix3 mArb = QuatMath.ToMatrix3(qArb);
        Check(Near(mArb.Determinant(), 1f, 1e-3f), "旋转矩阵行列式 = 1");
        Check(NearVec(mArb.MultiplyVec(new Vec3(1, 2, 3)), Quat.Rotate(qArb, new Vec3(1, 2, 3)), EPS),
              "矩阵旋转与四元数旋转一致");
        Check(SameRotation(QuatMath.FromMatrix3(mArb), qArb, EPS), "矩阵→四元数还原");
        Check(SameRotation(QuatMath.FromMatrix3(QuatMath.ToMatrix3(qx90)), qx90, EPS), "Rx90 矩阵往返还原");

        // 行正交性
        Matrix3 m90 = QuatMath.ToMatrix3(qz90);
        Vec3 row0 = new Vec3(m90.M00, m90.M01, m90.M02);
        Vec3 row1 = new Vec3(m90.M10, m90.M11, m90.M12);
        Check(Near(row0.Length, 1f, EPS) && Near(row1.Length, 1f, EPS), "矩阵行向量单位长");
        Check(Near(Vec3.Dot(row0, row1), 0f, EPS), "矩阵行向量正交");

        // ---- Slerp ----
        Check(SameRotation(QuatMath.Slerp(qx90, qy90, 0f), qx90, EPS), "Slerp t=0 = 起点");
        Check(SameRotation(QuatMath.Slerp(qx90, qy90, 1f), qy90, EPS), "Slerp t=1 = 终点");
        Check(Near(QuatMath.Slerp(qx90, qy90, 0.37f).Length, 1f, 1e-5f), "Slerp 中间结果单位长");

        // qx90 与 qy90 正交（点积=0）→ 中点 = 归一化 (a+b)
        Quat mid = QuatMath.Slerp(qx90, qy90, 0.5f);
        Quat midRef = (qx90 + qy90).Normalized();
        Check(SameRotation(mid, midRef, EPS), "Slerp 中点 = normalize(a+b)（正交情形）");

        // 匀速：等 t 间隔 → 等夹角
        float ang25 = QuatMath.AngleBetween(qx90, QuatMath.Slerp(qx90, qy90, 0.25f));
        float ang50 = QuatMath.AngleBetween(qx90, QuatMath.Slerp(qx90, qy90, 0.50f));
        // 球面角 = acos(dot)：总 60°；旋转差角 = 2×球面角 → t=0.25→30°、t=0.5→60°
        Check(Near(ang25, 30f, 1f) && Near(ang50, 60f, 1f) && Near(ang25 * 2f, ang50, 2f),
              "Slerp 匀速：旋转差角 t=0.25→30°、t=0.5→60°");

        // 最短弧：dot<0 自动取反
        Check(SameRotation(QuatMath.Slerp(qx90, -qy90, 0.5f), QuatMath.Slerp(qx90, qy90, 0.5f), EPS),
              "Slerp 沿最短弧（dot<0 取反等价）");

        // Lerp 不保证单位长
        float lenLerp = QuatMath.Lerp(qx90, qy90, 0.5f).Length;
        Check(lenLerp < 0.9999f, "Lerp 中点长度 < 1（非球面插值）");

        // 夹角
        Check(Near(QuatMath.AngleBetween(qx90, qx90), 0f, 1e-3f), "同向夹角 0");
        Check(Near(QuatMath.AngleBetween(Quat.Identity, qy180), 180f, 0.5f), "恒等与 Ry180 夹角 180");
        Check(Near(QuatMath.AngleBetween(qx90, qy90), 120f, 1f), "Rx90 与 Ry90 夹角 120（2*acos(0.5)）");

        // ---- SlerpPath ----
        List<Quat> path = QuatMath.SlerpPath(qx90, qy90, 24);
        Check(path.Count == 25, "SlerpPath 采样 24 步得 25 点");
        Check(SameRotation(path[0], qx90, EPS), "路径首点 = 起点");
        Check(SameRotation(path[24], qy90, EPS), "路径末点 = 终点");
        bool mono = true;
        for (int i = 1; i < 24; i++)
        {
            float a1 = QuatMath.AngleBetween(qx90, path[i - 1]);
            float a2 = QuatMath.AngleBetween(qx90, path[i]);
            if (a2 < a1 - 0.5f) mono = false;
        }
        Check(mono, "路径角距单调递增");

        // ---- 随机往返一致性 ----
        Random rnd = new Random(20261011);
        bool allOk = true;
        for (int i = 0; i < 200; i++)
        {
            float ang = (float)(rnd.NextDouble() * 360.0 - 180.0);
            Vec3 ax = new Vec3((float)rnd.NextDouble() * 2f - 1f,
                               (float)rnd.NextDouble() * 2f - 1f,
                               (float)rnd.NextDouble() * 2f - 1f);
            if (ax.IsZero) ax = new Vec3(0, 1, 0);
            Quat qq = QuatMath.FromAxisAngle(ax, ang);
            Vec3 ax2; float ang2;
            QuatMath.ToAxisAngle(qq, out ax2, out ang2);
            if (!SameRotation(QuatMath.FromAxisAngle(ax2, ang2), qq, 1e-3f)) allOk = false;

            float yE, pE, rE;
            QuatMath.ToEulerYXZ(qq, out yE, out pE, out rE);
            if (!SameRotation(QuatMath.FromEulerYXZ(yE, pE, rE), qq, 1e-3f)) allOk = false;

            Matrix3 mm = QuatMath.ToMatrix3(qq);
            if (!SameRotation(QuatMath.FromMatrix3(mm), qq, 1e-3f)) allOk = false;
        }
        Check(allOk, "随机 200 组：轴角/欧拉/矩阵 往返一致");

        // ---- 欧拉大范围往返（含负角与万向锁邻近）----
        bool eulerOk = true;
        float[,] eulerSet = new float[,]
        {
            { 170f, -120f, 45f },
            { -90f, 10f, -170f },
            { 33f, 89.5f, 120f },
            { -45f, -89.5f, 60f },
            { 0f, 89.999f, 0f },
        };
        for (int i = 0; i < 5; i++)
        {
            Quat qq = QuatMath.FromEulerYXZ(eulerSet[i, 0], eulerSet[i, 1], eulerSet[i, 2]);
            float yE, pE, rE;
            QuatMath.ToEulerYXZ(qq, out yE, out pE, out rE);
            if (!SameRotation(QuatMath.FromEulerYXZ(yE, pE, rE), qq, 1e-3f)) eulerOk = false;
        }
        Check(eulerOk, "欧拉大范围/万向锁邻近往返一致");

        Console.WriteLine();
        Console.WriteLine("RESULT: " + (failCount == 0 ? "ALL PASS" : (failCount + " FAIL")) +
                          "  (PASS " + passCount + ", FAIL " + failCount + ")");
        Environment.Exit(failCount == 0 ? 0 : 1);
    }
}
