using System.Collections.Generic;
using UnityEngine;
using QuatLab;

namespace QuatLab
{
    /// <summary>
    /// 四元数旋转变换交互演示主组件。
    ///
    /// 【演示内容】
    ///   - 一个六面异色立方体 + 局部坐标轴（X 红 / Y 绿 / Z 蓝），当前姿态由四元数 qCurrent 驱动
    ///   - 三种输入方式（可切换）：
    ///       轴-角      （轴仰角/方位角 + 旋转角度）
    ///       四元数     （w,x,y,z 滑条，实时归一化）
    ///       欧拉角 YXZ （yaw/pitch/roll，与 Unity Quaternion.Euler 应用顺序一致 Z→X→Y）
    ///   - 两种施加模式：
    ///       直接      —— 目标四元数立即生效
    ///       Slerp     —— 从当前姿态沿最短弧匀速转动到目标（球面线性插值）
    ///   - 最短弧轨迹：动画期间绘制三个基向量（X/Y/Z 局部轴）尖端扫过的弧线，
    ///     直观展示 Slerp 的匀速与最短弧性质；目标姿态以半透明“幽灵轴”显示
    ///   - 复合演示：右乘 = 局部系旋转，左乘 = 世界系旋转（四元数乘法不可交换）
    ///   - 实时读数：当前四元数 / 欧拉角 / 轴-角 / 3x3 旋转矩阵 / 与目标的夹角
    ///
    /// 【快捷键】Space 播放/暂停  R 重置  T 轨迹开关  H 幽灵轴/网格开关  F1 帮助  F 聚焦
    /// </summary>
    public class QuaternionLab : MonoBehaviour
    {
        public enum InputTab { AxisAngle, Quaternion, Euler }
        public enum DemoMode { Instant, Slerp }

        [Header("演示")]
        [Tooltip("Slerp 进度/秒（0.05~2）")]
        public float speed = 0.6f;
        [Range(16, 96)] public int pathSamples = 48;
        public float gizmoLength = 1.6f;
        public float cubeScale = 0.9f;

        [Header("输入（滑条状态）")]
        public InputTab tab = InputTab.AxisAngle;
        public DemoMode mode = DemoMode.Slerp;
        public float axisElev = 35f;      // 轴仰角（度，0=水平，90=竖直向上）
        public float axisAzim = 25f;      // 轴方位角（度，绕 Y）
        public float axisAngle = 60f;     // 旋转角（度）
        public float qW = 1f, qX = 0f, qY = 0f, qZ = 0f;
        public float eulerYaw = 0f, eulerPitch = 0f, eulerRoll = 0f;

        // ------------------------------------------------------------------
        // 内部状态
        // ------------------------------------------------------------------
        private Quat qCurrent;            // 当前显示姿态
        private Quat qTarget;             // 目标姿态（由输入计算）
        private Quat qTrailStart;         // 本次路径起点
        private bool animating;
        private float animT;
        private List<Quat> animPath = new List<Quat>();

        private GameObject cube;
        private LineRenderer axisX, axisY, axisZ;       // 当前局部轴
        private LineRenderer ghostX, ghostY, ghostZ;    // 目标幽灵轴
        private LineRenderer trailX, trailY, trailZ;    // 最短弧轨迹
        private GameObject gridRoot;

        private Camera cam;
        private bool showTrail = true;
        private bool showHelpers = true;
        private bool showHud = true;

        private readonly List<Material> materials = new List<Material>();

        // 配色
        private static readonly Color ColAxisX = new Color(1f, 0.22f, 0.22f);
        private static readonly Color ColAxisY = new Color(0.25f, 1f, 0.3f);
        private static readonly Color ColAxisZ = new Color(0.25f, 0.55f, 1f);
        private static readonly Color ColGhost = new Color(0.55f, 0.58f, 0.62f);
        private static readonly Color ColGrid = new Color(0.32f, 0.33f, 0.36f);

        private static readonly Vec3 BasisX = new Vec3(1f, 0f, 0f);
        private static readonly Vec3 BasisY = new Vec3(0f, 1f, 0f);
        private static readonly Vec3 BasisZ = new Vec3(0f, 0f, 1f);

        // ------------------------------------------------------------------
        // 生命周期
        // ------------------------------------------------------------------

        private void Start()
        {
            cam = Camera.main != null ? Camera.main : ObjectFinder.FindFirst<Camera>();
            CreateMaterials();
            CreateCube();
            CreateLocalAxes();
            CreateGhostAxes();
            CreateTrails();
            CreateGrid();

            qCurrent = Quat.Identity;
            qTarget = Quat.Identity;
            ComputeTargetFromInput();
            OnTargetChanged();
            // 启动即演示一次：从恒等 Slerp 到默认目标
            animating = true;
            animT = 0f;
        }

        private void OnDestroy()
        {
            for (int i = 0; i < materials.Count; i++)
            {
                if (materials[i] != null)
                    Destroy(materials[i]);
            }
            materials.Clear();
        }

        private void Update()
        {
            if (cam == null)
                cam = Camera.main != null ? Camera.main : ObjectFinder.FindFirst<Camera>();

            HandleKeys();

            // Slerp 动画推进
            if (animating && animPath.Count > 0)
            {
                animT += Time.deltaTime * speed;
                if (animT >= 1f)
                {
                    animT = 1f;
                    qCurrent = qTarget;
                    animating = false;
                }
                else
                {
                    int idx = Mathf.FloorToInt(animT * (animPath.Count - 1));
                    if (idx < 0) idx = 0;
                    if (idx >= animPath.Count) idx = animPath.Count - 1;
                    qCurrent = animPath[idx];
                }
            }
            else if (mode == DemoMode.Instant)
            {
                qCurrent = qTarget;
            }

            ApplyRotation();
            UpdateLocalAxes();
            UpdateGhostAxes();
            if (showTrail)
                UpdateTrails();
        }

        // ------------------------------------------------------------------
        // 目标输入与施加
        // ------------------------------------------------------------------

        private void ComputeTargetFromInput()
        {
            switch (tab)
            {
                case InputTab.AxisAngle:
                    qTarget = QuatMath.FromAxisAngle(AxisDir(), axisAngle);
                    break;
                case InputTab.Quaternion:
                    qTarget = new Quat(qW, qX, qY, qZ).Normalized();
                    // 把归一化结果写回滑条，保持显示一致
                    qW = qTarget.W; qX = qTarget.X; qY = qTarget.Y; qZ = qTarget.Z;
                    break;
                case InputTab.Euler:
                    qTarget = QuatMath.FromEulerYXZ(eulerYaw, eulerPitch, eulerRoll);
                    break;
            }
        }

        /// <summary>由仰角/方位角构造单位轴方向（竖直分量 = sin 仰角）。</summary>
        private Vec3 AxisDir()
        {
            float e = axisElev * Mathf.Deg2Rad;
            float a = axisAzim * Mathf.Deg2Rad;
            return new Vec3(
                Mathf.Cos(e) * Mathf.Sin(a),
                Mathf.Sin(e),
                Mathf.Cos(e) * Mathf.Cos(a)).Normalized();
        }

        /// <summary>输入变化：重算目标，按模式立即/动画施加，并重绘轨迹。</summary>
        private void OnTargetChanged()
        {
            qTrailStart = qCurrent;
            animPath = QuatMath.SlerpPath(qTrailStart, qTarget, pathSamples);
            if (mode == DemoMode.Slerp)
            {
                animating = true;
                animT = 0f;
            }
            else
            {
                qCurrent = qTarget;
                animating = false;
            }
            UpdateTrails();
            UpdateGhostAxes();
        }

        private void ApplyRotation()
        {
            if (cube == null)
                return;
            cube.transform.rotation = new Quaternion(qCurrent.W, qCurrent.X, qCurrent.Y, qCurrent.Z);
        }

        // ------------------------------------------------------------------
        // 快捷键
        // ------------------------------------------------------------------

        private void HandleKeys()
        {
            if (Input.GetKeyDown(KeyCode.Space))
                TogglePlay();
            if (Input.GetKeyDown(KeyCode.R))
                ResetDemo();
            if (Input.GetKeyDown(KeyCode.T))
                showTrail = !showTrail;
            if (Input.GetKeyDown(KeyCode.H))
                showHelpers = !showHelpers;
            if (Input.GetKeyDown(KeyCode.F1))
                showHud = !showHud;
            if (Input.GetKeyDown(KeyCode.F))
                FocusCamera();
        }

        private void TogglePlay()
        {
            if (animating)
            {
                animating = false;
            }
            else
            {
                // 若已到终点则从当前姿态重新向目标转（此时目标即当前，路径无变化）
                qTrailStart = qCurrent;
                animPath = QuatMath.SlerpPath(qTrailStart, qTarget, pathSamples);
                animating = true;
                animT = 0f;
                UpdateTrails();
            }
        }

        public void ResetDemo()
        {
            qTarget = Quat.Identity;
            qW = 1f; qX = 0f; qY = 0f; qZ = 0f;
            eulerYaw = 0f; eulerPitch = 0f; eulerRoll = 0f;
            axisElev = 35f; axisAzim = 25f; axisAngle = 60f;
            tab = InputTab.AxisAngle;
            OnTargetChanged();
        }

        // ------------------------------------------------------------------
        // 复合演示（左右乘）
        // ------------------------------------------------------------------

        /// <summary>右乘：在局部坐标系中叠加旋转（q' = q * R）。</summary>
        public void ComposeLocalRy90()
        {
            Quat ry = QuatMath.FromAxisAngle(BasisY, 90f);
            qTarget = qTarget * ry;
            OnTargetChanged();
        }

        /// <summary>左乘：在世界坐标系中叠加旋转（q' = R * q）。</summary>
        public void ComposeWorldRy90()
        {
            Quat ry = QuatMath.FromAxisAngle(BasisY, 90f);
            qTarget = ry * qTarget;
            OnTargetChanged();
        }

        /// <summary>快速预设：绕指定轴旋转 90°（轴为局部轴，等效目标姿态 = R）。</summary>
        public void SetRotationAbout(Vec3 axis)
        {
            qTarget = QuatMath.FromAxisAngle(axis, 90f);
            OnTargetChanged();
        }

        // ------------------------------------------------------------------
        // 数据辅助
        // ------------------------------------------------------------------

        private static UnityEngine.Quaternion ToUnity(Quat q)
        {
            return new UnityEngine.Quaternion(q.W, q.X, q.Y, q.Z);
        }

        private static Vector3 ToUnity(Vec3 v)
        {
            return new Vector3(v.X, v.Y, v.Z);
        }

        /// <summary>演示对象包围盒（相机聚焦用）。</summary>
        public Bounds LabBounds
        {
            get
            {
                float r = gizmoLength * 1.3f + cubeScale;
                return new Bounds(Vector3.zero, Vector3.one * (r * 2f));
            }
        }

        public void FocusCamera()
        {
            OrbitCameraController orbit = ObjectFinder.FindFirst<OrbitCameraController>();
            if (orbit != null)
                orbit.FocusOn(LabBounds);
        }



        // ------------------------------------------------------------------
        // 场景物件创建
        // ------------------------------------------------------------------

        private void CreateMaterials()
        {
            // 立方体六面（顺序：+Z 蓝, -Z 青, +Y 绿, -Y 黄, +X 红, -X 紫）
            MakeUnlit(new Color(0.2f, 0.45f, 1f));
            MakeUnlit(new Color(0.2f, 0.9f, 0.9f));
            MakeUnlit(new Color(0.25f, 1f, 0.3f));
            MakeUnlit(new Color(1f, 0.85f, 0.2f));
            MakeUnlit(new Color(1f, 0.22f, 0.22f));
            MakeUnlit(new Color(0.85f, 0.25f, 0.95f));
        }

        private void CreateCube()
        {
            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = "RotatingCube";
            Collider col = cube.GetComponent<Collider>();
            if (col != null)
                Destroy(col);
            cube.transform.SetParent(transform, false);
            cube.transform.localScale = Vector3.one * cubeScale;

            MeshRenderer mr = cube.GetComponent<MeshRenderer>();
            if (mr != null)
            {
                // 六面异色：沿用上面创建的 6 个材质
                Material[] mats = new Material[materials.Count];
                for (int i = 0; i < materials.Count; i++)
                    mats[i] = materials[i];
                mr.sharedMaterials = mats;
            }
        }

        private void CreateLocalAxes()
        {
            axisX = MakeLineGo("LocalAxisX", MakeUnlit(ColAxisX), 0.035f, transform);
            axisY = MakeLineGo("LocalAxisY", MakeUnlit(ColAxisY), 0.035f, transform);
            axisZ = MakeLineGo("LocalAxisZ", MakeUnlit(ColAxisZ), 0.035f, transform);
        }

        private void CreateGhostAxes()
        {
            ghostX = MakeLineGo("GhostAxisX", MakeUnlit(ColGhost), 0.02f, transform);
            ghostY = MakeLineGo("GhostAxisY", MakeUnlit(ColGhost), 0.02f, transform);
            ghostZ = MakeLineGo("GhostAxisZ", MakeUnlit(ColGhost), 0.02f, transform);
        }

        private void CreateTrails()
        {
            trailX = MakeLineGo("TrailX", MakeUnlit(new Color(1f, 0.45f, 0.45f)), 0.05f, transform);
            trailY = MakeLineGo("TrailY", MakeUnlit(new Color(0.45f, 1f, 0.5f)), 0.05f, transform);
            trailZ = MakeLineGo("TrailZ", MakeUnlit(new Color(0.45f, 0.7f, 1f)), 0.05f, transform);
        }

        private void CreateGrid()
        {
            if (gridRoot != null)
                return;
            gridRoot = new GameObject("Grid");
            gridRoot.transform.SetParent(transform, false);

            Material gridMat = MakeUnlit(ColGrid);
            for (int i = -4; i <= 4; i++)
            {
                MakeLineGo("GridX_" + i, gridMat, 0.01f, gridRoot.transform);
                MakeLineGo("GridZ_" + i, gridMat, 0.01f, gridRoot.transform);
            }

            MakeLineGo("WorldAxisX", MakeUnlit(ColAxisX), 0.025f, gridRoot.transform);
            MakeLineGo("WorldAxisY", MakeUnlit(ColAxisY), 0.025f, gridRoot.transform);
            MakeLineGo("WorldAxisZ", MakeUnlit(ColAxisZ), 0.025f, gridRoot.transform);
        }

        private LineRenderer MakeLineGo(string name, Material mat, float width, Transform parent)
        {
            GameObject go = new GameObject(name);
            go.transform.SetParent(parent, false);
            LineRenderer lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.sharedMaterial = mat;
            lr.startWidth = width;
            lr.endWidth = width;
            lr.positionCount = 2;
            lr.SetPosition(0, Vector3.zero);
            lr.SetPosition(1, Vector3.zero);
            return lr;
        }

        private Material MakeUnlit(Color c)
        {
            Shader sh = Shader.Find("Unlit/Color");
            if (sh == null)
                sh = Shader.Find("Standard");
            Material m = new Material(sh);
            if (m.HasProperty("_Color"))
                m.SetColor("_Color", c);
            materials.Add(m);
            return m;
        }

        private static void SetLine(LineRenderer lr, Vector3 a, Vector3 b)
        {
            if (lr == null)
                return;
            lr.positionCount = 2;
            lr.SetPosition(0, a);
            lr.SetPosition(1, b);
        }

        // ------------------------------------------------------------------
        // 每帧几何更新
        // ------------------------------------------------------------------

        private void UpdateLocalAxes()
        {
            float len = gizmoLength;
            SetLine(axisX, Vector3.zero, ToUnity(Quat.Rotate(qCurrent, BasisX)) * len);
            SetLine(axisY, Vector3.zero, ToUnity(Quat.Rotate(qCurrent, BasisY)) * len);
            SetLine(axisZ, Vector3.zero, ToUnity(Quat.Rotate(qCurrent, BasisZ)) * len);
        }

        private void UpdateGhostAxes()
        {
            bool show = showHelpers;
            float len = gizmoLength * 1.25f;
            SetLine(ghostX, Vector3.zero, ToUnity(Quat.Rotate(qTarget, BasisX)) * len);
            SetLine(ghostY, Vector3.zero, ToUnity(Quat.Rotate(qTarget, BasisY)) * len);
            SetLine(ghostZ, Vector3.zero, ToUnity(Quat.Rotate(qTarget, BasisZ)) * len);
            if (ghostX != null) ghostX.gameObject.SetActive(show);
            if (ghostY != null) ghostY.gameObject.SetActive(show);
            if (ghostZ != null) ghostZ.gameObject.SetActive(show);
            if (gridRoot != null) gridRoot.SetActive(show);
        }

        /// <summary>绘制 Slerp 路径上三个基向量尖端扫过的弧线。</summary>
        private void UpdateTrails()
        {
            if (animPath == null || animPath.Count == 0)
                return;
            FillTrail(trailX, BasisX);
            FillTrail(trailY, BasisY);
            FillTrail(trailZ, BasisZ);
        }

        private void FillTrail(LineRenderer lr, Vec3 basis)
        {
            if (lr == null)
                return;
            float len = gizmoLength;
            lr.positionCount = animPath.Count;
            Vector3[] pts = new Vector3[animPath.Count];
            for (int i = 0; i < animPath.Count; i++)
                pts[i] = ToUnity(Quat.Rotate(animPath[i], basis)) * len;
            lr.SetPositions(pts);
        }



        // ------------------------------------------------------------------
        // HUD（IMGUI）
        // ------------------------------------------------------------------

        private void OnGUI()
        {
            if (!showHud)
                return;

            GUILayout.BeginArea(new Rect(12, 12, 360, Screen.height - 40));
            GUILayout.BeginVertical(GUI.skin.box);

            GUIStyle title = new GUIStyle(GUI.skin.label);
            title.fontStyle = FontStyle.Bold;
            title.fontSize = 15;
            GUILayout.Label("四元数旋转变换演示", title);

            // 施加模式
            GUILayout.BeginHorizontal();
            if (GUILayout.Toggle(mode == DemoMode.Instant, "直接", GUI.skin.button) && mode != DemoMode.Instant)
            {
                mode = DemoMode.Instant;
                OnTargetChanged();
            }
            if (GUILayout.Toggle(mode == DemoMode.Slerp, "Slerp 动画", GUI.skin.button) && mode != DemoMode.Slerp)
            {
                mode = DemoMode.Slerp;
                OnTargetChanged();
            }
            if (GUILayout.Button(animating ? "暂停 (Space)" : "播放 (Space)"))
                TogglePlay();
            GUILayout.EndHorizontal();

            // 输入方式
            GUILayout.Label("输入方式：");
            GUILayout.BeginHorizontal();
            if (GUILayout.Toggle(tab == InputTab.AxisAngle, "轴-角", GUI.skin.button) && tab != InputTab.AxisAngle)
            {
                tab = InputTab.AxisAngle;
                ComputeTargetFromInput();
                OnTargetChanged();
            }
            if (GUILayout.Toggle(tab == InputTab.Quaternion, "四元数", GUI.skin.button) && tab != InputTab.Quaternion)
            {
                tab = InputTab.Quaternion;
                ComputeTargetFromInput();
                OnTargetChanged();
            }
            if (GUILayout.Toggle(tab == InputTab.Euler, "欧拉 YXZ", GUI.skin.button) && tab != InputTab.Euler)
            {
                tab = InputTab.Euler;
                ComputeTargetFromInput();
                OnTargetChanged();
            }
            GUILayout.EndHorizontal();

            // 输入面板
            switch (tab)
            {
                case InputTab.AxisAngle:
                    SliderLabel("轴仰角", ref axisElev, -89f, 89f, "°");
                    SliderLabel("轴方位角", ref axisAzim, -180f, 180f, "°");
                    SliderLabel("旋转角", ref axisAngle, -360f, 360f, "°");
                    break;
                case InputTab.Quaternion:
                    SliderLabel("w（实部）", ref qW, -1f, 1f, "");
                    SliderLabel("x", ref qX, -1f, 1f, "");
                    SliderLabel("y", ref qY, -1f, 1f, "");
                    SliderLabel("z", ref qZ, -1f, 1f, "");
                    break;
                case InputTab.Euler:
                    SliderLabel("yaw（绕Y）", ref eulerYaw, -180f, 180f, "°");
                    SliderLabel("pitch（绕X）", ref eulerPitch, -180f, 180f, "°");
                    SliderLabel("roll（绕Z）", ref eulerRoll, -180f, 180f, "°");
                    break;
            }
            if (GUILayout.Button("应用输入（重新计算目标）"))
            {
                ComputeTargetFromInput();
                OnTargetChanged();
            }

            // 复合演示
            GUILayout.Label("复合（四元数乘法，不可交换）：");
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("右乘 Ry90（局部系）")) ComposeLocalRy90();
            if (GUILayout.Button("左乘 Ry90（世界系）")) ComposeWorldRy90();
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("绕 X 90°")) SetRotationAbout(BasisX);
            if (GUILayout.Button("绕 Y 90°")) SetRotationAbout(BasisY);
            if (GUILayout.Button("绕 Z 90°")) SetRotationAbout(BasisZ);
            if (GUILayout.Button("重置 (R)")) ResetDemo();
            GUILayout.EndHorizontal();

            // 显示开关与速度
            GUILayout.BeginHorizontal();
            showTrail = GUILayout.Toggle(showTrail, "最短弧轨迹 (T)");
            showHelpers = GUILayout.Toggle(showHelpers, "幽灵轴/网格 (H)");
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("速度", GUILayout.Width(36));
            speed = GUILayout.HorizontalSlider(speed, 0.05f, 2f);
            GUILayout.Label(speed.ToString("F2"), GUILayout.Width(40));
            GUILayout.EndHorizontal();

            // 实时读数
            GUILayout.Space(4);
            GUIStyle mono = new GUIStyle(GUI.skin.label);
            mono.fontSize = 11;
            GUILayout.Label("当前姿态：");
            GUILayout.Label("  四元数 q = " + qCurrent.ToString(), mono);
            float yawD, pitchD, rollD;
            QuatMath.ToEulerYXZ(qCurrent, out yawD, out pitchD, out rollD);
            GUILayout.Label("  欧拉   = (" + Fmt(yawD) + ", " + Fmt(pitchD) + ", " + Fmt(rollD) + ")°", mono);
            Vec3 axCur;
            float angCur;
            QuatMath.ToAxisAngle(qCurrent, out axCur, out angCur);
            GUILayout.Label("  轴-角  = 轴" + axCur.ToString() + "  角度 " + Fmt(angCur) + "°", mono);
            Matrix3 m3 = QuatMath.ToMatrix3(qCurrent);
            GUILayout.Label("  矩阵   = " + m3.ToString().Replace("\n", "\n            "), mono);
            float diff = QuatMath.AngleBetween(qCurrent, qTarget);
            GUILayout.Label("  与目标夹角 = " + Fmt(diff) + "°   进度 = " + (animT * 100f).ToString("F0") + "%", mono);

            // 帮助
            GUILayout.Space(6);
            GUIStyle help = new GUIStyle(GUI.skin.label);
            help.fontSize = 11;
            help.wordWrap = true;
            GUILayout.Label(
                "说明：\n" +
                "· 立方体+局部轴 = 当前姿态；灰幽灵轴 = 目标姿态\n" +
                "· 彩色弧线 = Slerp 最短弧上基向量尖端轨迹\n" +
                "· 右乘 = 局部系旋转；左乘 = 世界系旋转（不可交换）\n" +
                "· 输入改动点“应用”生效；Slerp 模式自动播放\n" +
                "· Space 播放/暂停  R 重置  T 轨迹  H 辅助\n" +
                "· F1 隐藏本面板  F 聚焦  | 相机：右键旋转 中键平移 滚轮缩放",
                help);

            GUILayout.EndVertical();
            GUILayout.EndArea();
        }

        private void SliderLabel(string label, ref float value, float lo, float hi, string unit)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(92));
            float before = value;
            value = GUILayout.HorizontalSlider(value, lo, hi);
            GUILayout.Label(Fmt(value) + unit, GUILayout.Width(64));
            GUILayout.EndHorizontal();
            if (Mathf.Abs(value - before) > 0.0001f)
            {
                ComputeTargetFromInput();
                OnTargetChanged();
            }
        }

        private static string Fmt(float v)
        {
            return v.ToString("F1");
        }
    }
}
