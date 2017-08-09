﻿using System;
using System.Collections.Generic;
using System.Linq;
using Equinox.ProceduralWorld.Buildings.Creation;
using Equinox.ProceduralWorld.Buildings.Exporter;
using Equinox.ProceduralWorld.Buildings.Game;
using Equinox.ProceduralWorld.Buildings.Generation;
using Equinox.ProceduralWorld.Buildings.Library;
using Equinox.ProceduralWorld.Buildings.Seeds;
using Equinox.ProceduralWorld.Buildings.Storage;
using Equinox.Utils;
using Equinox.Utils.Command;
using Equinox.Utils.Logging;
using Equinox.Utils.Session;
using Sandbox.Definitions;
using Sandbox.ModAPI;
using VRage;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;

namespace Equinox.ProceduralWorld.Buildings
{
    public class MyBuildingControlCommands : MyCommandProviderComponent
    {
        private MyPartManager m_partManager;

        public MyBuildingControlCommands()
        {
            DependsOn((MyPartManager x) => m_partManager = x);
            if (!SessionCore.RELEASE)
            {
                Create("clear").Handler(ClearStations);
                Create("spawn").AllowOnlyOn(MySessionType.PlayerController).Handler(ProcessSpawn)
                    .NamedFlag("debug").NamedArgument<int?>("rooms", null).NamedArgument<long>("seed", 0).NamedArgument<int?>(new[] { "pop", "population" }, null);
                Create("info").AllowOnlyOn(MySessionType.PlayerController).Handler<string>(ProcessInfo);
                Create("stations").Handler(ProcessStationLocations);
                Create("part").Handler<string>(ProcessDebugPart);
            }
            Create("export").PromotedOnly(MyPromoteLevel.Admin).Handler(RunExport);
        }

        private string RunExport()
        {
            MyAPIGateway.Entities.GetEntities(null, x =>
            {
                var grid = x as IMyCubeGrid;
                if (grid != null)
                    MyDesignTool.Process(grid);
                return false;
            });
            return null;
        }

        private string ClearStations()
        {
            var id = MyAPIGateway.Session.Player.IdentityId;
            foreach (var gps in MyAPIGateway.Session.GPS.GetGpsList(id))
                MyAPIGateway.Session.GPS.RemoveGps(id, gps);
            var ent = new HashSet<IMyEntity>();
            MyAPIGateway.Entities.GetEntities(ent, (x) => x is IMyCubeGrid);
            foreach (var k in ent)
                MyAPIGateway.Entities.RemoveEntity(k);
            return null;
        }

        private string ProcessStationLocations()
        {
            var stationModule = Manager.GetDependencyProvider<MyProceduralStationModule>();
            if (stationModule == null)
                return "No station module means no stations";
            var factionModule = Manager.GetDependencyProvider<MyProceduralFactions>();
            if (factionModule == null)
                return "No faction module means no stations";
            var stationNoise = stationModule.StationNoise;
            var sphere = new BoundingSphereD(MyAPIGateway.Session.Camera.Position, MyAPIGateway.Session.SessionSettings.ViewDistance * 10);
            var id = MyAPIGateway.Session.Player.IdentityId;
            foreach (var gps in MyAPIGateway.Session.GPS.GetGpsList(id))
                if (gps.Name.StartsWith("Station - "))
                    MyAPIGateway.Session.GPS.RemoveGps(id, gps);
            var aabb = new BoundingBoxD(sphere.Center - sphere.Radius, sphere.Center + sphere.Radius);
            foreach (var s in stationNoise.TryGetSpawnIn(aabb, sphere.Intersects))
            {
                var position = new Vector3D(s.Item2.X, s.Item2.Y, s.Item2.Z);
                var cseed = new MyProceduralConstructionSeed(factionModule.SeedAt(s.Item2.XYZ()), s.Item2, null, s.Item1.GetHashCode());
                var gps = MyAPIGateway.Session.GPS.Create("[" + cseed.Faction.Tag + "] " + cseed.Name, "", position, true, true);
                gps.DiscardAt = MyAPIGateway.Session.ElapsedPlayTime + new TimeSpan(0, 5, 0);
                MyAPIGateway.Session.GPS.AddGps(id, gps);
            }
            return null;
        }

        private string ProcessInfo(string partName)
        {
            var part = m_partManager.FirstOrDefault(test => test.Prefab.Id.SubtypeName.ToLower().Contains(partName.ToLower()));
            if (part == null)
                return "Unable to find part with name \"" + partName + "\"";
            var info = part.BlockSetInfo;
            Log(MyLogSeverity.Info, "Part info for {0}\nBlock type counts:", part.Name);
            foreach (var kv in info.BlockCountByType)
                Log(MyLogSeverity.Info, "{0}: {1}", kv.Key, kv.Value);
            Log(MyLogSeverity.Info, "Total power consumption/storage: {0:e}:{1:e}  Groups:", info.TotalPowerNetConsumption, info.TotalPowerStorage);
            foreach (var kv in info.PowerConsumptionByGroup)
                Log(MyLogSeverity.Info, "{0}: {1}", kv.Key, kv.Value);
            Log(MyLogSeverity.Info, "Total inventory capacity: {0:e}", info.TotalInventoryCapacity);
            Log(MyLogSeverity.Info, "Total component cost:");
            foreach (var kv in info.ComponentCost)
                Log(MyLogSeverity.Info, "{0}: {1}", kv.Key.Id.SubtypeName, kv.Value);
            Log(MyLogSeverity.Info, "Production quotas:");
            foreach (var pi in MyDefinitionManager.Static.GetPhysicalItemDefinitions())
                Log(MyLogSeverity.Info, "{0}: {1}", pi.Id, info.TotalProduction(pi.Id));
            foreach (var pi in MyDefinitionManager.Static.GetDefinitionsOfType<MyComponentDefinition>())
                Log(MyLogSeverity.Info, "{0}: {1}", pi.Id, info.TotalProduction(pi.Id));
            foreach (var gas in MyDefinitionManager.Static.GetDefinitionsOfType<MyGasProperties>())
                Log(MyLogSeverity.Info, "{0}: {1}", gas.Id, info.TotalProduction(gas.Id));
            Log(MyLogSeverity.Info, "Gas storage");
            foreach (var gas in MyDefinitionManager.Static.GetDefinitionsOfType<MyGasProperties>())
                Log(MyLogSeverity.Info, "{0}: {1}", gas.Id, info.TotalGasStorage(gas.Id));
            return null;
        }

        private string ProcessDebugPart(string partName)
        {
            var part = m_partManager.FirstOrDefault(test => test.Prefab.Id.SubtypeName.ToLower().Contains(partName.ToLower()));
            if (part == null)
                return "Unable to find part with name \"" + partName + "\"";
            var position = MyAPIGateway.Session.Camera.Position + MyAPIGateway.Session.Camera.WorldMatrix.Forward * 100;
            var seed = new MyProceduralConstructionSeed(new MyProceduralFactionSeed("dummy", 0), new Vector4D(position, 0.5), null, 0);

            MyAPIGateway.Parallel.Start(() =>
            {
                var construction = new MyProceduralConstruction(seed);
                var room = new MyProceduralRoom();
                room.Init(new MatrixI(Base6Directions.Direction.Forward, Base6Directions.Direction.Up), part);
                construction.AddRoom(room);
                var remapper = new MyRoomRemapper { DebugRoomColors = true };
                var grids = MyGridCreator.RemapAndBuild(construction, remapper);
                if (grids == null) return;
                MyAPIGateway.Utilities.InvokeOnGameThread(() =>
                {
                    var component = grids.SpawnAsync();
                    if (component != null)
                        component.ForceDebugDraw = true;
                });
            });
            return null;
        }

        private string ProcessSpawn(Dictionary<string, object> kwargs)
        {
            var generatorModule = Manager.GetDependencyProvider<MyStationGeneratorManager>();
            if (generatorModule == null)
                return "No station generator module means no stations";
            var factionModule = Manager.GetDependencyProvider<MyProceduralFactions>();
            if (factionModule == null)
                return "No faction module means no stations";

            var debugMode = (bool)kwargs["debug"];
            var roomCount = (int?) kwargs["rooms"];
            var seedVal = (long) kwargs["seed"];
            var population = (int) kwargs["population"];
            var position = MyAPIGateway.Session.Camera.Position + MyAPIGateway.Session.Camera.WorldMatrix.Forward * 100;
            var seed = new MyProceduralConstructionSeed(factionModule.SeedAt(position), new Vector4D(position, 0.5), null, seedVal, population);
            MyAPIGateway.Parallel.Start(() =>
            {
                MyProceduralConstruction construction = null;
                MyConstructionCopy grids;
                if (!generatorModule.GenerateFromSeedAndRemap(seed, ref construction, out grids, roomCount))
                {
                    this.Error("Failed to generate");
                    return;
                }
                if (grids == null)
                {
                    this.Error("Failed to generate: Output grids are null");
                    return;
                }
                MyAPIGateway.Utilities.InvokeOnGameThread(() =>
                {
                    var result = grids.SpawnAsync();
                    result.ForceDebugDraw |= debugMode;
                });
            });
            return null;
        }

        public override void LoadConfiguration(MyObjectBuilder_ModSessionComponent config)
        {
            if (config == null) return;
            if (config is MyObjectBuilder_BuildingControlCommands) return;
            Log(MyLogSeverity.Critical, "Configuration type {0} doesn't match component type {1}", config.GetType(), GetType());
        }

        public override MyObjectBuilder_ModSessionComponent SaveConfiguration()
        {
            return new MyObjectBuilder_BuildingControlCommands();
        }
    }

    public class MyObjectBuilder_BuildingControlCommands : MyObjectBuilder_ModSessionComponent
    {
    }
}
