using System;
using UnityEngine;

namespace FluidSim2D
{
    public class FluidSim2D : MonoBehaviour
    {
        public readonly static int THREAD_NUM = 32;

        [SerializeField] protected ComputeShader _computeShader;
        [SerializeField] protected float _timeStep = 0.125f;
        [SerializeField] protected float _impulseTemperature = 10f;
        [SerializeField] protected float _impulseDensity = 1f;
        [SerializeField] protected float _temperatureDissipation = 0.99f;
        [SerializeField] protected float _velocityDissipation = 0.99f;
        [SerializeField] protected float _densityDissipation = 0.9999f;
        [SerializeField] protected float _ambientTemperature = 0f;
        [SerializeField] protected float _smokeBuoyancy = 1f;
        [SerializeField] protected float _smokeWeight = 0.05f;
        [SerializeField] protected float _cellSize = 1f;
        [SerializeField] protected float _gradientScale = 1f;
        [SerializeField] protected int _numJacobiIterations = 50;

        protected RenderTexture[] _velocity = new RenderTexture[2];
        protected RenderTexture[] _temperature = new RenderTexture[2];
        protected RenderTexture[] _density = new RenderTexture[2];
        protected RenderTexture[] _pressure = new RenderTexture[2];
        protected RenderTexture _divergence;
        protected RenderTexture _obstacles;
        protected RenderTexture _impulse;

        protected Vector2Int _resolution = new Vector2Int(512, 512);
        protected Vector2 _resolutionFloat;
        protected Vector4 _texelSize;

        protected int _advectKernelID;
        protected int _buoyancyKernelID;
        protected int _impulseKernelID;
        protected int _divergenceKernelID;
        protected int _jacobiKernelID;
        protected int _subtractGradientKernelID;
        protected int _texelSizeID;
        protected int _timeStepID;
        protected int _sourceID;
        protected int _velocityID;
        protected int _obstaclesID;
        protected int _impulseID;
        protected int _outputID;
        protected int _temperatureID;
        protected int _densityID;
        protected int _divergenceID;
        protected int _pressureID;
        protected int _dissipationID;
        protected int _ambientTemperatureID;
        protected int _sigmaID;
        protected int _kappaID;
        protected int _halfInverseCellSizeID;
        protected int _alphaID;
        protected int _inverseBetaID;
        protected int _gradientScaleID;
        protected int _fillId;

        public float TimeStep { get => _timeStep; set => _timeStep = value; }
        public float ImpulseTemperature { get => _impulseTemperature; set => _impulseTemperature = value; }
        public float ImpulseDensity { get => _impulseDensity; set => _impulseDensity = value; }
        public float TemperatureDissipation { get => _temperatureDissipation; set => _temperatureDissipation = value; }
        public float VelocityDissipation { get => _velocityDissipation; set => _velocityDissipation = value; }
        public float DensityDissipation { get => _densityDissipation; set => _densityDissipation = value; }
        public float AmbientTemperature { get => _ambientTemperature; set => _ambientTemperature = value; }
        public float SmokeBuoyancy { get => _smokeBuoyancy; set => _smokeBuoyancy = value; }
        public float SmokeWeight { get => _smokeWeight; set => _smokeWeight = value; }
        public float CellSize { get => _cellSize; set => _cellSize = value; }
        public float GradientScale { get => _gradientScale; set => _gradientScale = value; }
        public int NumJacobiIterations { get => _numJacobiIterations; set => _numJacobiIterations = value; }
        public RenderTexture Velocity { get => _velocity[0]; }
        public RenderTexture Temperature { get => _temperature[0]; }
        public RenderTexture Density { get => _density[0]; }
        public RenderTexture Pressure { get => _pressure[0]; }
        public RenderTexture Divergence { get => _divergence; }
        public RenderTexture Obstacles { get => _obstacles; }
        public RenderTexture Impulse { get => _impulse; }
        public Vector2Int Resolution
        {
            get => _resolution;
            set
            {
                _resolution = value;
                _resolutionFloat = _resolution;
                _texelSize = new Vector4(1f / _resolutionFloat.x, 1f / _resolutionFloat.y, _resolutionFloat.x, _resolutionFloat.y);
                InitRenderTexture();
            }
        }
        public Vector2 ResolutionFloat { get => _resolutionFloat; }

        protected virtual void Awake()
        {
            InitKernel();
            InitShaderID();
        }

        protected virtual void Start()
        {
            _resolutionFloat = _resolution;
            _texelSize = new Vector4(1f / _resolutionFloat.x, 1f / _resolutionFloat.y, _resolutionFloat.x, _resolutionFloat.y);
            InitRenderTexture();
        }

        private const int READ = 0;
        private const int WRITE = 1;

        protected virtual void Update()
        {
            ComputeAdvect(_velocity[READ], _velocity[READ], _velocity[WRITE], _velocityDissipation, _timeStep);
            ComputeAdvect(_velocity[READ], _temperature[READ], _temperature[WRITE], _temperatureDissipation, _timeStep);
            ComputeAdvect(_velocity[READ], _density[READ], _density[WRITE], _densityDissipation, _timeStep);
            (_velocity[READ], _velocity[WRITE]) = (_velocity[WRITE], _velocity[READ]);
            (_temperature[READ], _temperature[WRITE]) = (_temperature[WRITE], _temperature[READ]);
            (_density[READ], _density[WRITE]) = (_density[WRITE], _density[READ]);

            ComputeBuoyancy(_velocity[READ], _temperature[READ], _density[READ], _velocity[WRITE], _timeStep);
            (_velocity[READ], _velocity[WRITE]) = (_velocity[WRITE], _velocity[READ]);

            ComputeImpulse(_temperature[READ], _temperature[WRITE], _impulseTemperature);
            ComputeImpulse(_density[READ], _density[WRITE], _impulseDensity);
            (_temperature[READ], _temperature[WRITE]) = (_temperature[WRITE], _temperature[READ]);
            (_density[READ], _density[WRITE]) = (_density[WRITE], _density[READ]);

            ComputeDivergence(_velocity[READ], _divergence);

            ClearTexture(_pressure[READ]);

            for (int i = 0; i < _numJacobiIterations; i++)
            {
                ComputeJacobi(_pressure[READ], _divergence, _pressure[WRITE]);
                (_pressure[READ], _pressure[WRITE]) = (_pressure[WRITE], _pressure[READ]);
            }

            ComputeSubtractGradient(_velocity[READ], _pressure[READ], _velocity[WRITE]);
            (_velocity[READ], _velocity[WRITE]) = (_velocity[WRITE], _velocity[READ]);
        }

        protected virtual void OnDestroy()
        {
            ReleaseRenderTextures();
        }

        protected virtual void ComputeAdvect(RenderTexture velocity, RenderTexture source, RenderTexture output, float dissipation, float timeStep)
        {
            _computeShader.SetVector(_texelSizeID, _texelSize);
            _computeShader.SetFloat(_timeStepID, timeStep);
            _computeShader.SetFloat(_dissipationID, dissipation);
            _computeShader.SetTexture(_advectKernelID, _velocityID, velocity);
            _computeShader.SetTexture(_advectKernelID, _sourceID, source);
            _computeShader.SetTexture(_advectKernelID, _obstaclesID, _obstacles);
            _computeShader.SetTexture(_advectKernelID, _outputID, output);

            _computeShader.Dispatch(_advectKernelID, _resolution.x / THREAD_NUM, _resolution.y / THREAD_NUM, 1);
        }

        protected virtual void ComputeBuoyancy(RenderTexture velocity, RenderTexture temperature, RenderTexture density, RenderTexture output, float timeStep)
        {
            _computeShader.SetFloat(_ambientTemperatureID, _ambientTemperature);
            _computeShader.SetFloat(_timeStepID, timeStep);
            _computeShader.SetFloat(_sigmaID, _smokeBuoyancy);
            _computeShader.SetFloat(_kappaID, _smokeWeight);
            _computeShader.SetTexture(_buoyancyKernelID, _velocityID, velocity);
            _computeShader.SetTexture(_buoyancyKernelID, _temperatureID, temperature);
            _computeShader.SetTexture(_buoyancyKernelID, _densityID, density);
            _computeShader.SetTexture(_buoyancyKernelID, _outputID, output);

            _computeShader.Dispatch(_buoyancyKernelID, _resolution.x / THREAD_NUM, _resolution.y / THREAD_NUM, 1);
        }

        protected virtual void ComputeImpulse(RenderTexture source, RenderTexture output, float val)
        {
            _computeShader.SetFloat(_fillId, val);
            _computeShader.SetTexture(_impulseKernelID, _sourceID, source);
            _computeShader.SetTexture(_impulseKernelID, _impulseID, _impulse);
            _computeShader.SetTexture(_impulseKernelID, _outputID, output);

            _computeShader.Dispatch(_impulseKernelID, _resolution.x / THREAD_NUM, _resolution.y / THREAD_NUM, 1);
        }

        protected virtual void ComputeDivergence(RenderTexture velocity, RenderTexture output)
        {
            _computeShader.SetFloat(_halfInverseCellSizeID, 0.5f / _cellSize);
            _computeShader.SetTexture(_divergenceKernelID, _velocityID, velocity);
            _computeShader.SetTexture(_divergenceKernelID, _obstaclesID, _obstacles);
            _computeShader.SetTexture(_divergenceKernelID, _outputID, output);

            _computeShader.Dispatch(_divergenceKernelID, _resolution.x / THREAD_NUM, _resolution.y / THREAD_NUM, 1);
        }

        protected virtual void ComputeJacobi(RenderTexture pressure, RenderTexture divergence, RenderTexture output)
        {
            _computeShader.SetFloat(_alphaID, -_cellSize * _cellSize);
            _computeShader.SetFloat(_inverseBetaID, 0.25f);
            _computeShader.SetTexture(_jacobiKernelID, _pressureID, pressure);
            _computeShader.SetTexture(_jacobiKernelID, _divergenceID, divergence);
            _computeShader.SetTexture(_jacobiKernelID, _obstaclesID, _obstacles);
            _computeShader.SetTexture(_jacobiKernelID, _outputID, output);

            _computeShader.Dispatch(_jacobiKernelID, _resolution.x / THREAD_NUM, _resolution.y / THREAD_NUM, 1);
        }

        protected virtual void ComputeSubtractGradient(RenderTexture velocity, RenderTexture pressure, RenderTexture output)
        {
            _computeShader.SetFloat(_gradientScaleID, _gradientScale);
            _computeShader.SetTexture(_subtractGradientKernelID, _velocityID, velocity);
            _computeShader.SetTexture(_subtractGradientKernelID, _pressureID, pressure);
            _computeShader.SetTexture(_subtractGradientKernelID, _obstaclesID, _obstacles);
            _computeShader.SetTexture(_subtractGradientKernelID, _outputID, output);

            _computeShader.Dispatch(_subtractGradientKernelID, _resolution.x / THREAD_NUM, _resolution.y / THREAD_NUM, 1);
        }

        protected virtual void ClearTexture(RenderTexture rt)
        {
            Graphics.SetRenderTarget(rt);
            GL.Clear(false, true, new Color(0, 0, 0, 0));
            Graphics.SetRenderTarget(null);
        }

        protected virtual void InitKernel()
        {
            _advectKernelID = _computeShader.FindKernel("compute_advect");
            _buoyancyKernelID = _computeShader.FindKernel("compute_buoyancy");
            _impulseKernelID = _computeShader.FindKernel("compute_impulse");
            _divergenceKernelID = _computeShader.FindKernel("compute_divergence");
            _jacobiKernelID = _computeShader.FindKernel("compute_jacobi");
            _subtractGradientKernelID = _computeShader.FindKernel("compute_subtract_gradient");
        }

        protected virtual void InitShaderID()
        {
            _sourceID = Shader.PropertyToID("source");
            _velocityID = Shader.PropertyToID("velocity");
            _obstaclesID = Shader.PropertyToID("obstacles");
            _impulseID = Shader.PropertyToID("impulse");
            _temperatureID = Shader.PropertyToID("temperature");
            _densityID = Shader.PropertyToID("density");
            _divergenceID = Shader.PropertyToID("divergence");
            _pressureID = Shader.PropertyToID("pressure");
            _outputID = Shader.PropertyToID("output");
            _texelSizeID = Shader.PropertyToID("texel_size");
            _timeStepID = Shader.PropertyToID("time_step");
            _dissipationID = Shader.PropertyToID("dissipation");
            _ambientTemperatureID = Shader.PropertyToID("ambient_temperature");
            _sigmaID = Shader.PropertyToID("sigma");
            _kappaID = Shader.PropertyToID("kappa");
            _halfInverseCellSizeID = Shader.PropertyToID("half_inverse_cell_size");
            _alphaID = Shader.PropertyToID("alpha");
            _inverseBetaID = Shader.PropertyToID("inverse_beta");
            _gradientScaleID = Shader.PropertyToID("gradient_scale");
            _fillId = Shader.PropertyToID("fill");
        }

        protected virtual void InitRenderTexture()
        {
            ReleaseRenderTextures();

            _velocity[0] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RGFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _velocity[1] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RGFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _temperature[0] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _temperature[1] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _density[0] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _density[1] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _pressure[0] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _pressure[1] = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _divergence = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _obstacles = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
            _impulse = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.RFloat, RenderTextureReadWrite.Linear)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                enableRandomWrite = true
            };
        }

        protected virtual void ReleaseRenderTextures()
        {
            _velocity[0]?.Release();
            _velocity[1]?.Release();
            _temperature[0]?.Release();
            _temperature[1]?.Release();
            _density[0]?.Release();
            _density[1]?.Release();
            _pressure[0]?.Release();
            _pressure[1]?.Release();
            _divergence?.Release();
            _obstacles?.Release();
            _impulse?.Release();
        }
    }
}