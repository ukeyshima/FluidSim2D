using UnityEngine;

namespace FluidSim2D.Sample
{
    public class ResolutionSetter : MonoBehaviour
    {
        [SerializeField] private Vector2Int _resolution = new Vector2Int(512, 512);
        [SerializeField] private FluidSim2D _fluidSim2D;
        [SerializeField] private Obstacles _obstacles;
        [SerializeField] private Impulse _impulse;

        private Vector2Int _resolutionPre;

        private void Awake()
        {
            SetResolution();
            _resolutionPre = _resolution;
        }

        private void Update()
        {
            if (_resolution != _resolutionPre)
            {
                SetResolution();
                _resolutionPre = _resolution;
            }
        }

        private void SetResolution()
        {
            _fluidSim2D.Resolution = _resolution;
            _obstacles.Resolution = _resolution;
            _impulse.Resolution = _resolution;
        }
    }
}