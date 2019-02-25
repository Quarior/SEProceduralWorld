using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Equinox.ProceduralWorld.Manager;
using Equinox.ProceduralWorld.Voxels.VoxelBuilder;
using Equinox.Utils;
using Equinox.Utils.Logging;
using Equinox.Utils.Random;
using Equinox.Utils.Session;
using Sandbox.Definitions;
using Sandbox.Game.Entities;
using Sandbox.Game.World.Generator;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Utils;
using VRageMath;

namespace Equinox.ProceduralWorld.Voxels.Planets
{
    public class InfinitePlanetsModule : ProceduralModule
    {
        private Ob_InfinitePlanets _config = new Ob_InfinitePlanets();
        private readonly Dictionary<Vector3I, ProceduralSystem> _systems = new Dictionary<Vector3I, ProceduralSystem>();
        private readonly Queue<ProceduralBody> _addQueue = new Queue<ProceduralBody>();


        public override IEnumerable<ProceduralObject> Generate(BoundingSphereD include, BoundingSphereD? exclude)
        {
            var mult = _config.ViewDistance / include.Radius;
            include.Radius *= mult;
            if (exclude.HasValue)
                exclude = new BoundingSphereD(exclude.Value.Center, exclude.Value.Radius * mult);

            var min = Vector3D.Floor((include.Center - include.Radius) / _config.SystemSpacing);
            var max = Vector3D.Floor(((include.Center + include.Radius) / _config.SystemSpacing) + 1.0D);
            for (var x = min.X; x < max.X; x++)
                for (var y = min.Y; y < max.Y; y++)
                    for (var z = min.Z; z < max.Z; z++)
                    {
                        var seedVec = new Vector3I(x, y, z);
                        var seed = seedVec.GetHashCode();
                        var rand = new Random(seed);
                        if (rand.NextDouble() >= _config.SystemProbability)
                            continue;
                        if (_systems.ContainsKey(seedVec))
                            continue;
                        var world = (new Vector3D(x, y, z) + 0.5) * _config.SystemSpacing + rand.NextVector3D() * (_config.SystemSpacing / 8);
                        if (include.Contains(world) == ContainmentType.Disjoint || (exclude.HasValue && exclude.Value.Contains(world) != ContainmentType.Disjoint))
                            continue;

                        var list = _config.Systems.Where(s => s.MinDistanceFromOrigin <= world.Length()).ToList();
                        var ttl = list.Sum(s => s.Probability);
                        foreach (var s in list)
                        {
                            if (ttl <= s.Probability)
                            {
                                var position = MatrixD.CreateFromQuaternion(rand.NextQuaternion());
                                position.Translation = world;
                                var result = new ProceduralSystem(this, s, rand.NextLong(), position);
                                _systems.Add(seedVec, result);
                                foreach (var b in result.Bodies)
                                    _addQueue.Enqueue(b);
                                yield return result;
                                break;
                            }
                            ttl -= s.Probability;
                        }
                    }
        }

        public override bool TickAfterSimulationRoundRobin()
        {
            ProceduralBody body;
            if (_addQueue.TryDequeue(out body))
            {
                var generatorDef =
                    MyDefinitionManagerBase.Static.GetDefinition<MyPlanetGeneratorDefinition>(body.GeneratorId);
                body.Result = VoxelUtility.SpawnPlanet(body.Position.Translation, generatorDef, body.Seed, (float)body.Radius, body.Name,
                    (float)body.Gravity, body.GravityRelative, (float)body.GravityFalloff, body.AddGps, body.SpherizeWithDistance);
                return true;
            }
            return false;
        }

        public override void LoadConfiguration(Ob_ModSessionComponent config)
        {
            var ob = config as Ob_InfinitePlanets;
            if (ob == null)
            {
                Log(MyLogSeverity.Critical, "Configuration type {0} doesn't match component type {1}", config.GetType(), GetType());
                return;
            }
            _config = MyAPIGateway.Utilities.SerializeFromXML<Ob_InfinitePlanets>(MyAPIGateway.Utilities.SerializeToXML(config));
        }

        public override Ob_ModSessionComponent SaveConfiguration()
        {
            return MyAPIGateway.Utilities.SerializeFromXML<Ob_InfinitePlanets>(MyAPIGateway.Utilities.SerializeToXML(_config));
        }

        public override bool RunOnClients => false;

        private class ProceduralBody
        {
            public MyDefinitionId GeneratorId;
            public double Radius;
            public MatrixD Position;
            public MyPlanet Result;
            public long Seed;
            public string Name;
            public double Gravity;
            public bool GravityRelative;
            public double GravityFalloff;
            public bool AddGps;
            public bool SpherizeWithDistance;
        }

        private class ProceduralSystem : ProceduralObject
        {
            public readonly List<ProceduralBody> Bodies = new List<ProceduralBody>();

            private struct MoonBuilderInfo
            {
                public Ob_InfinitePlanets_MoonDesc Desc;
                public double OrbitRadius;
                public double BodyRadius;
                public double Gravity;
                public bool GravityRelative;
                public double GravityFalloff;
                public bool AddGps;
                public bool SpherizeWithDistance;
            }

            private struct Moon2BuilderInfo
            {
                public Ob_InfinitePlanets_Moon2Desc Desc;
                public double OrbitRadius;
                public double BodyRadius;
                public double Gravity;
                public bool GravityRelative;
                public double GravityFalloff;
                public bool AddGps;
                public bool SpherizeWithDistance;
            }

            private static T SampleBodies<T>(IReadOnlyCollection<T> input, Random rand, double currentRadius) where T : Ob_InfinitePlanets_BodyDesc
            {
                var canidates = input.Where(x => x.OrbitRadius.Max > currentRadius).ToList();
                if (canidates.Count == 0)
                    return null;
                var lks = rand.NextDouble() *
                          canidates.Sum(x => x.Probability);
                var result = canidates[0];
                foreach (var t in canidates)
                {
                    var prob = t.Probability;
                    if (lks <= prob)
                    {
                        result = t;
                        break;
                    }
                    lks -= prob;
                }
                return result;
            }

            public ProceduralSystem(InfinitePlanetsModule module, Ob_InfinitePlanets_SystemDesc desc, long seed, MatrixD position)
                : base(module)
            {
                var rand = new Random((int)(seed >> 32) ^ (int)(seed));

                var planetCount = (int) desc.PlanetCount.Sample(rand);
                var currentRadius = 0D;
                var moonBuffer = new List<MoonBuilderInfo>();
                var moon2Buffer = new List<Moon2BuilderInfo>();
                module.Info("Generating system {0} at {1}.  Target planet count is {2}.", rand, position.Translation, planetCount);
                module.IncreaseIndent();
                for (var planetId = 0; planetId < planetCount; planetId++)
                {
                    Ob_InfinitePlanets_PlanetDesc planet = SampleBodies(desc.PlanetTypes, rand, currentRadius);
                    while (planet != null && currentRadius < planet.OrbitRadius.Min)
                    {
                        if (currentRadius < planet.OrbitRadius.Min)
                            currentRadius = Math.Min(planet.OrbitRadius.Min, 2 * currentRadius + Math.Sqrt(planet.OrbitRadius.Min));
                        planet = SampleBodies(desc.PlanetTypes, rand, currentRadius);
                    }
                    if (planet == null)
                        break;
                    currentRadius = Math.Max(currentRadius, planet.OrbitRadius.Min);

                    var planetRadius = planet.BodyRadius.Sample(rand);
                    var planetGravity = planet.Gravity.Sample(rand);
                    var currentMoonRadius = planetRadius * 2 + planet.MoonSpacing.Sample(rand);
                    moonBuffer.Clear();
                    {
                        var moonCount = planet.MoonCount.Sample(rand);
                        while (moonCount > 0)
                        {
                            Ob_InfinitePlanets_MoonDesc moon =
                                SampleBodies(planet.MoonTypes, rand, currentMoonRadius);
                            if (moon == null)
                                break;

                            var moonRadius = moon.BodyRadius.Sample(rand);
                            var moonGravity = moon.Gravity.Sample(rand);
                            moonBuffer.Add(new MoonBuilderInfo()
                            {
                                BodyRadius = moonRadius,
                                Desc = moon,
                                Gravity = moonGravity,
                                GravityRelative = moon.GravityRelative,
                                GravityFalloff = moon.GravityFalloff,
                                SpherizeWithDistance = moon.SpherizeWithDistance,
                                AddGps = moon.AddGps,
                                OrbitRadius = currentMoonRadius
                            }
                            );
                            
                            var moon2Count = moon.Moon2Count.Sample(rand);
                            var currentMoon2Radius = moonRadius * 2 + moon.Moon2Spacing.Sample(rand);
                            moon2Buffer.Clear();
                            {
                                while (moon2Count > 0)
                                {
                                    Ob_InfinitePlanets_Moon2Desc moon2 =
                                        SampleBodies(moon.Moon2Types, rand, currentMoon2Radius);
                                    if (moon2 == null)
                                        break;

                                    var moon2Radius = moon2.BodyRadius.Sample(rand);
                                    var moon2Gravity = moon2.Gravity.Sample(rand);
                                    moon2Buffer.Add(new Moon2BuilderInfo()
                                    {
                                        BodyRadius = moon2Radius,
                                        Desc = moon2,
                                        Gravity = moon2Gravity,
                                        GravityRelative = moon2.GravityRelative,
                                        GravityFalloff = moon2.GravityFalloff,
                                        SpherizeWithDistance = moon2.SpherizeWithDistance,
                                        AddGps = moon2.AddGps,
                                        OrbitRadius = currentMoon2Radius
                                    }
                                    );
                                    currentMoon2Radius += moon2Radius;
                                    currentMoon2Radius += moon.Moon2Spacing.Sample(rand);
                                    moon2Count--;
                                }
                            }

                            currentMoonRadius += moonRadius;
                            currentMoonRadius += planet.MoonSpacing.Sample(rand);
                            moonCount--;
                        }
                    }

                    currentRadius += currentMoonRadius;

                    var orbitalPlane = MatrixD.CreateRotationX(planet.OrbitInclinationDeg.Sample(rand) * (Math.PI / 180D)) * position;
                    var planetPosition = CreateXZDir(planet.OrbitLocationDeg.Sample(rand) * (Math.PI / 180D)) *
                                         currentRadius;
                    orbitalPlane = MatrixD.CreateTranslation(planetPosition) * orbitalPlane;

                    Bodies.Add(new ProceduralBody()
                    {
                        GeneratorId = planet.Generator,
                        Position = orbitalPlane,
                        Radius = planetRadius,
                        Result = null,
                        Seed = rand.NextLong(),
                        Name = string.Format("sys_{0:X16}_{1:X2}", seed, planetId+1),
                        Gravity = planetGravity,
                        GravityRelative = planet.GravityRelative,
                        GravityFalloff = planet.GravityFalloff,
                        AddGps = planet.AddGps,
                        SpherizeWithDistance = planet.SpherizeWithDistance
        });
                    module.Info("- {0} w/ radius {1}, orbiting at {2}, at {3}.  Target moon count is {4}.", planet.Generator.SubtypeName, planetRadius, currentRadius, orbitalPlane.Translation, moonBuffer.Count);
                    module.IncreaseIndent();
                    for (var moonId = 0; moonId < moonBuffer.Count; moonId++)
                    {
                        var moon = moonBuffer[moonId];
                        var moonPlane =
                            MatrixD.CreateRotationX(moon.Desc.OrbitInclinationDeg.Sample(rand) * (Math.PI / 180D)) *
                            orbitalPlane;
                        var moonPosition = CreateXZDir(moon.Desc.OrbitLocationDeg.Sample(rand) * (Math.PI / 180D)) *
                                           moon.OrbitRadius;
                        moonPlane = MatrixD.CreateTranslation(moonPosition) * moonPlane;
                        module.Info("+ {0} w/ radius {1}, orbiting at {2}, at {3}.  Target moon count is {4}.", moon.Desc.Generator.SubtypeName, moon.BodyRadius, moon.OrbitRadius, moonPlane.Translation, moon2Buffer.Count);
                        Bodies.Add(new ProceduralBody()
                        {
                            GeneratorId = moon.Desc.Generator,
                            Radius = moon.BodyRadius,
                            Position = moonPlane,
                            Result = null,
                            Seed = rand.NextLong(),
                            Name = string.Format("sys_{0:X16}_{1:X2}_{2:X2}", seed, planetId+1, moonId+1),
                            Gravity = moon.Gravity,
                            GravityRelative = moon.GravityRelative,
                            GravityFalloff = moon.GravityFalloff,
                            AddGps = moon.AddGps,
                            SpherizeWithDistance = moon.SpherizeWithDistance
                        });

                        module.IncreaseIndent();
                        for (var moon2Id = 0; moon2Id < moon2Buffer.Count; moon2Id++)
                        {
                            var moon2 = moon2Buffer[moon2Id];
                            var moon2Plane =
                                MatrixD.CreateRotationX(moon2.Desc.OrbitInclinationDeg.Sample(rand) * (Math.PI / 180D)) *
                                moonPlane;
                            var moon2Position = CreateXZDir(moon2.Desc.OrbitLocationDeg.Sample(rand) * (Math.PI / 180D)) *
                                               moon2.OrbitRadius;
                            moon2Plane = MatrixD.CreateTranslation(moon2Position) * moon2Plane;
                            module.Info("++ {0} w/ radius {1}, orbiting at {2}, at {3}", moon2.Desc.Generator.SubtypeName, moon2.BodyRadius, moon2.OrbitRadius, moon2Plane.Translation);
                            Bodies.Add(new ProceduralBody()
                            {
                                GeneratorId = moon2.Desc.Generator,
                                Radius = moon2.BodyRadius,
                                Position = moon2Plane,
                                Result = null,
                                Seed = rand.NextLong(),
                                Name = string.Format("sys_{0:X16}_{1:X2}_{2:X2}_{3:X2}", seed, planetId+1, moonId+1, moon2Id+1),
                                Gravity = moon2.Gravity,
                                GravityRelative = moon2.GravityRelative,
                                GravityFalloff = moon2.GravityFalloff,
                                AddGps = moon2.AddGps,
                                SpherizeWithDistance = moon2.SpherizeWithDistance
                            });
                        }
                        module.DecreaseIndent();

                    }
                    module.DecreaseIndent();

                    currentRadius += currentMoonRadius;
                    currentRadius += desc.PlanetSpacing.Sample(rand);
                }
                module.DecreaseIndent();
            }

            private static Vector3D CreateXZDir(double theta)
            {
                return new Vector3D(Math.Cos(theta), 0, Math.Sin(theta));
            }
        }
    }
}
