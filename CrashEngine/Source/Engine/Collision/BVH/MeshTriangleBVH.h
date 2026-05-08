// 충돌/피킹 영역에서 공유되는 타입과 인터페이스를 정의합니다.
#pragma once

#include "Collision/BVH/BVH.h"
#include "Core/CoreTypes.h"
#include "Core/CollisionTypes.h"
#include "Core/EngineTypes.h"
#include "Math/Vector.h"

#include "Collision/RayUtilsSIMD.h"
#include "Math/Intersection.h"
#include "Mesh/StaticMeshAsset.h"
#include <bit>
#include <cfloat>

struct FStaticMesh;

namespace
{
    // 메시 내부 BVH는 world BVH보다 leaf가 작고 traversal stack도 더 얕게 유지합니다.
    constexpr int32 MeshBVHMaxTraversalStack = 256;

    // 삼각형 한 개를 감싸는 local-space AABB를 만듭니다.
    FBoundingBox MakeTriangleBounds(const FVector& V0, const FVector& V1, const FVector& V2)
    {
        FBoundingBox Bounds;
        Bounds.Expand(V0);
        Bounds.Expand(V1);
        Bounds.Expand(V2);
        return Bounds;
    }

} // namespace

// FMeshTriangleBVH 클래스이다.
class FMeshTriangleBVH
{
public:
    static constexpr int32 ChildFanout = 8;
    static constexpr int32 MaxLeafSize = 8;

    // 메시의 모든 삼각형을 leaf로 수집한 뒤 로컬 공간 BVH를 즉시 다시 빌드합니다.
    template <typename MeshType>
    void BuildNow(const MeshType& Mesh);
    // 현재는 static mesh asset이 로드 후 고정된다고 보고, 아직 트리가 없을 때만 1회 빌드합니다.
    template <typename MeshType>
    void EnsureBuilt(const MeshType& Mesh);
    // 로컬 공간 ray로 BVH를 순회해 가장 가까운 삼각형 hit를 찾습니다.
    template <typename MeshType>
    bool RaycastLocal(const FVector& LocalOrigin,
                      const FVector& LocalDirection,
                      const MeshType& Mesh,
                      FHitResult& OutHitResult) const;

    // 월드 primitive BVH와 dirty 플래그 의미가 겹쳐 혼동을 줄 수 있어,
    // 메시 변경 대응용 API는 일단 주석으로 남겨 둡니다.
    // void MarkDirty();

private:
    // FTriangleLeaf는 충돌/피킹 처리에 필요한 데이터를 묶는 구조체입니다.
    struct FTriangleLeaf
    {
        FBoundingBox Bounds;
        // Mesh.Indices에서 이 삼각형이 시작하는 위치입니다.
        int32 TriangleStartIndex = 0;
    };

    using FBVH = TBVH<FTriangleLeaf, ChildFanout, MaxLeafSize>;
    using FNode = FBVH::FNode;

    // alignas는 충돌/피킹 처리에 필요한 데이터를 묶는 구조체입니다.
    struct alignas(32) FTrianglePacket
    {
        alignas(32) float V0X[8];
        alignas(32) float V0Y[8];
        alignas(32) float V0Z[8];
        alignas(32) float Edge1X[8];
        alignas(32) float Edge1Y[8];
        alignas(32) float Edge1Z[8];
        alignas(32) float Edge2X[8];
        alignas(32) float Edge2Y[8];
        alignas(32) float Edge2Z[8];
        int32 TriangleStartIndices[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
        uint32 LaneMask = 0;
        uint32 TriangleCount = 0;
    };

    // 현재는 메시가 런타임 중 변하지 않는다고 보고, dirty 기반 재빌드는 비활성화합니다.
    // 오브젝트가 새로 로드되어 트리가 비어 있을 경우에만 EnsureBuilt에서 빌드합니다.
    // bool bDirty = true;\

    // 삼각형 단위 leaf 배열입니다. 오브젝트들을 BVH 리프 노드에서처럼 여기서는 이 배열의 연속 구간을 가리킵니다.
    TArray<FTriangleLeaf> TriangleLeaves;
    TArray<FTrianglePacket> LeafPackets;
    FBVH BVH;
};

template <typename MeshType>
void FMeshTriangleBVH::BuildNow(const MeshType& Mesh)
{
    // 메시가 바뀌었을 때 triangle leaf와 packet, node 배열을 통째로 다시 만듭니다.
    TriangleLeaves.clear();
    LeafPackets.clear();
    BVH.Reset();

    if (Mesh.Vertices.empty() || Mesh.Indices.size() < 3)
    {
        return;
    }

    TriangleLeaves.reserve(Mesh.Indices.size() / 3);
    LeafPackets.reserve((Mesh.Indices.size() / 3 + 7) / 8);

    // 인덱스 버퍼를 삼각형 단위로 훑으면서 각 삼각형의 bounds와 시작 인덱스를 leaf로 만듭니다.
    for (size_t Index = 0; Index + 2 < Mesh.Indices.size(); Index += 3)
    {
        const FVector& V0 = Mesh.Vertices[Mesh.Indices[Index]].Position;
        const FVector& V1 = Mesh.Vertices[Mesh.Indices[Index + 1]].Position;
        const FVector& V2 = Mesh.Vertices[Mesh.Indices[Index + 2]].Position;

        FTriangleLeaf Leaf;
        Leaf.TriangleStartIndex = static_cast<int32>(Index);
        Leaf.Bounds = MakeTriangleBounds(V0, V1, V2);
        TriangleLeaves.push_back(Leaf);
    }

    if (!TriangleLeaves.empty())
    {
        BVH.Build(
            TriangleLeaves,
            [](const FTriangleLeaf& Leaf) { return Leaf.Bounds; },
            [](const FTriangleLeaf& Leaf) { return Leaf.TriangleStartIndex; });

        const TArray<FNode>& Nodes = BVH.GetNodes();
        for (const FNode& Node : Nodes)
        {
            if (!Node.IsLeaf())
            {
                continue;
            }

            FNode& MutableNode = const_cast<FNode&>(Node);
            MutableNode.PayloadIndex = static_cast<int32>(LeafPackets.size());
            MutableNode.PayloadCount = 1;

            FTrianglePacket Packet{};
            Packet.TriangleCount = static_cast<uint32>(Node.LeafCount);

            for (int32 Index = 0; Index < ChildFanout; ++Index)
            {
                if (Index < Node.LeafCount)
                {
                    const int32 TriStartIndex = TriangleLeaves[Node.FirstLeaf + Index].TriangleStartIndex;
                    const FVector& V0 = Mesh.Vertices[Mesh.Indices[TriStartIndex]].Position;
                    const FVector& V1 = Mesh.Vertices[Mesh.Indices[TriStartIndex + 1]].Position;
                    const FVector& V2 = Mesh.Vertices[Mesh.Indices[TriStartIndex + 2]].Position;
                    const FVector Edge1 = V1 - V0;
                    const FVector Edge2 = V2 - V0;

                    Packet.V0X[Index] = V0.X;
                    Packet.V0Y[Index] = V0.Y;
                    Packet.V0Z[Index] = V0.Z;
                    Packet.Edge1X[Index] = Edge1.X;
                    Packet.Edge1Y[Index] = Edge1.Y;
                    Packet.Edge1Z[Index] = Edge1.Z;
                    Packet.Edge2X[Index] = Edge2.X;
                    Packet.Edge2Y[Index] = Edge2.Y;
                    Packet.Edge2Z[Index] = Edge2.Z;
                    Packet.TriangleStartIndices[Index] = TriStartIndex;
                    Packet.LaneMask |= (1u << Index);
                }
                else
                {
                    Packet.V0X[Index] = Packet.V0Y[Index] = Packet.V0Z[Index] = 0.0f;
                    Packet.Edge1X[Index] = Packet.Edge1Y[Index] = Packet.Edge1Z[Index] = 0.0f;
                    Packet.Edge2X[Index] = Packet.Edge2Y[Index] = Packet.Edge2Z[Index] = 0.0f;
                }
            }

            LeafPackets.push_back(Packet);
        }
    }
}



template <typename MeshType>
void FMeshTriangleBVH::EnsureBuilt(const MeshType& Mesh)
{
    // static mesh asset은 로드 후 고정된다고 보고, 아직 비어 있을 때만 1회 빌드합니다.
    if (!BVH.GetNodes().empty())
    {
        return;
    }
    BuildNow(Mesh);
}

template <typename MeshType>
bool FMeshTriangleBVH::RaycastLocal(const FVector& LocalOrigin, const FVector& LocalDirection, const MeshType& Mesh, FHitResult& OutHitResult) const
{
    // 로컬 공간 ray로 메시 BVH를 front-to-back 순회하면서 가장 가까운 삼각형 hit를 찾습니다
    struct FTraversalEntry
    {
        int32 NodeIndex;
        float TMin;
    };

    OutHitResult = {};
    const TArray<FNode>& Nodes = BVH.GetNodes();
    if (Nodes.empty() || Mesh.Vertices.empty() || Mesh.Indices.size() < 3)
    {
        return false;
    }

    FTraversalEntry NodeStack[MeshBVHMaxTraversalStack];
    int32 StackSize = 0;

    // world 단계에서 이미 world transform을 벗겼으므로 여기서는 local ray만 다루면 됩니다.
    FRay LocalRay;
    LocalRay.Origin = LocalOrigin;
    LocalRay.Direction = LocalDirection;

    float RootTMin = 0.0f;
    float RootTMax = 0.0f;
    if (!FMath::IntersectRayAABB(LocalRay, Nodes[0].Bounds, RootTMin, RootTMax))
    {
        return false;
    }

    const FRaySIMDContext RayContext = FRayUtilsSIMD::MakeRayContext(LocalOrigin, LocalDirection);

    // 재귀 대신 고정 크기 스택으로 순회해 call overhead와 allocation을 피합니다.
    NodeStack[StackSize++] = { 0, RootTMin };

    float ClosestT = FLT_MAX;
    bool bHit = false;

    while (StackSize > 0)
    {
        const FTraversalEntry Entry = NodeStack[--StackSize];
        if (Entry.TMin > ClosestT)
        {
            continue;
        }

        const FNode& Node = Nodes[Entry.NodeIndex];

        if (Node.IsLeaf())
        {
            // leaf는 최대 8개 triangle을 SoA packet으로 묶어 두었기 때문에
            // raycast 시에는 triangle 교차를 packet 단위 SIMD 한 번으로 처리할 수 있습니다.
            const FTrianglePacket& Packet = LeafPackets[Node.PayloadIndex];

            alignas(32) float TValues[8];
            const int32 Mask = FRayUtilsSIMD::IntersectTriangles8Precomputed(
                RayContext,
                Packet.V0X, Packet.V0Y, Packet.V0Z,
                Packet.Edge1X, Packet.Edge1Y, Packet.Edge1Z,
                Packet.Edge2X, Packet.Edge2Y, Packet.Edge2Z,
                ClosestT,
                TValues) &
                static_cast<int32>(Packet.LaneMask);

            if (Mask != 0)
            {
                // hit lane만 비트 스캔으로 순회해 scalar 후처리 비용을 줄인다.
                uint32 RemainingMask = static_cast<uint32>(Mask);
                while (RemainingMask != 0)
                {
                    const uint32 Lane = std::countr_zero(RemainingMask);
                    if (TValues[Lane] < ClosestT)
                    {
                        ClosestT = TValues[Lane];
                        OutHitResult.bHit = true;
                        OutHitResult.Distance = TValues[Lane];
                        OutHitResult.FaceIndex = Packet.TriangleStartIndices[Lane];
                        bHit = true;
                    }
                    RemainingMask &= (RemainingMask - 1);
                }
            }
            continue;
        }

        // internal node에서는 child AABB 8개를 한 번에 검사한 뒤,
        // 실제로 맞은 child만 추려 가까운 순서대로 방문합니다.
        alignas(32) float TMinValues[8];
        const int32 NodeMask = FRayUtilsSIMD::IntersectAABB8(
            RayContext,
            Node.ChildMinX, Node.ChildMinY, Node.ChildMinZ,
            Node.ChildMaxX, Node.ChildMaxY, Node.ChildMaxZ,
            ClosestT,
            TMinValues);
        if (NodeMask == 0)
        {
            continue;
        }

        FTraversalEntry ChildEntries[8];
        int32 ChildEntryCount = 0;
        // 자식 수는 최대 8개라서 hit child만 모아도 정렬 비용이 작고,
        // 가까운 순서대로 방문하면 ClosestT가 빨리 줄어 후속 prune이 잘 됩니다
        uint32 RemainingMask = static_cast<uint32>(NodeMask) & ((1u << Node.ChildCount) - 1u);
        while (RemainingMask != 0)
        {
            const uint32 Lane = std::countr_zero(RemainingMask);
            ChildEntries[ChildEntryCount++] = { Node.Children[Lane], TMinValues[Lane] };
            RemainingMask &= (RemainingMask - 1);
        }

        if (ChildEntryCount == 1)
        {
            if (StackSize < MeshBVHMaxTraversalStack)
            {
                NodeStack[StackSize++] = ChildEntries[0];
            }
            continue;
        }

        if (ChildEntryCount == 2 && ChildEntries[0].TMin < ChildEntries[1].TMin)
        {
            std::swap(ChildEntries[0], ChildEntries[1]);
        }
        else if (ChildEntryCount > 2)
        {
            for (int32 I = 1; I < ChildEntryCount; ++I)
            {
                FTraversalEntry Key = ChildEntries[I];
                int32 J = I - 1;
                while (J >= 0 && ChildEntries[J].TMin < Key.TMin)
                {
                    ChildEntries[J + 1] = ChildEntries[J];
                    --J;
                }
                ChildEntries[J + 1] = Key;
            }
        }

        for (int32 I = 0; I < ChildEntryCount && StackSize < MeshBVHMaxTraversalStack; ++I)
        {
            NodeStack[StackSize++] = ChildEntries[I];
        }
    }

    return bHit;
}
