#include "SkeletalMesh.h"

#include "SkeletalMeshAsset.h"

IMPLEMENT_CLASS(USkeletalMesh, UObject)

static const FString EmptyPath;

USkeletalMesh::~USkeletalMesh()
{
    if (SkeletalMeshAsset)
    {
        const uint32 CPUSize =
            static_cast<uint32>(SkeletalMeshAsset->Vertices.size() * sizeof(FVertexPNCT_T)) +
            static_cast<uint32>(SkeletalMeshAsset->Indices.size() * sizeof(uint32));

        MemoryStats::SubStaticMeshCPUMemory(CPUSize);
    }
}

void USkeletalMesh::Serialize(FArchive& Ar)
{
    UObject::Serialize(Ar);

    if (Ar.IsLoading() && !SkeletalMeshAsset)
    {
        SkeletalMeshAsset = new FSkeletalMesh();
    }

    SkeletalMeshAsset->Serialize(Ar);

    Ar << Materials;

    if (Ar.IsLoading())
    {
        for (FSkeletalMeshSection& Section : SkeletalMeshAsset->Sections)
        {
            Section.MaterialIndex = -1;
            for (int32 i = 0; i < (int32)Materials.size(); ++i)
            {
                if (Materials[i].MaterialSlotName == Section.MaterialSlotName)
                {
                    Section.MaterialIndex = i;
                    break;
                }
            }
        }
    }
}

const FString& USkeletalMesh::GetAssetPathFileName() const
{
    if (SkeletalMeshAsset)
    {
        return SkeletalMeshAsset->PathFileName;
    }
    return EmptyPath;
}

void USkeletalMesh::SetSkeletalMeshAsset(FSkeletalMesh* InMesh)
{
    SkeletalMeshAsset = InMesh;

    if (SkeletalMeshAsset)
    {
        for (FSkeletalMeshSection& Section : SkeletalMeshAsset->Sections)
        {
            Section.MaterialIndex = -1;
            for (int32 i = 0; i < (int32)Materials.size(); ++i)
            {
                if (Materials[i].MaterialSlotName == Section.MaterialSlotName)
                {
                    Section.MaterialIndex = i;
                    break;
                }
            }
        }
        EnsureMeshTrianglePickingBVHBuilt();
    }
}

FSkeletalMesh* USkeletalMesh::GetSkeletalMeshAsset() const
{
    return SkeletalMeshAsset;
}

void USkeletalMesh::SetMaterials(TArray<FMeshMaterial>&& InMaterials)
{
    Materials = InMaterials;
}


const TArray<FMeshMaterial>& USkeletalMesh::GetMaterials() const
{
    return Materials;
}

void USkeletalMesh::InitResources(ID3D11Device* InDevice)
{
    if (!InDevice || !SkeletalMeshAsset)
        return;

    // CPU 메모리 추적
    const uint32 CPUSize =
        static_cast<uint32>(SkeletalMeshAsset->Vertices.size() * sizeof(FSkeletalMeshVertex)) +
        static_cast<uint32>(SkeletalMeshAsset->Indices.size() * sizeof(uint32));
    MemoryStats::AddSkeletalMeshCPUMemory(CPUSize);

    // CPU → GPU 정점 버퍼 변환
    TMeshData<FSkeletalMeshVertex> RenderMeshData;
    RenderMeshData.Vertices.reserve(SkeletalMeshAsset->Vertices.size());

    for (const FSkeletalMeshVertex& RawVert : SkeletalMeshAsset->Vertices)
    {
        FSkeletalMeshVertex RenderVert;
        RenderVert.Position = RawVert.Position;
        RenderVert.Normal = RawVert.Normal;
        RenderVert.Color = RawVert.Color;
        RenderVert.UV = RawVert.UV;
        RenderVert.Tangent = RawVert.Tangent;
        
        std::memcpy(RenderVert.BoneIndices, RawVert.BoneIndices, sizeof(int32) * 4);
        std::memcpy(RenderVert.BoneWeights, RawVert.BoneWeights, sizeof(float) * 4);

        RenderMeshData.Vertices.push_back(RenderVert);
    }
    RenderMeshData.Indices = SkeletalMeshAsset->Indices;

    SkeletalMeshAsset->RenderBuffer = std::make_unique<FDynamicMeshBuffer>();
    SkeletalMeshAsset->RenderBuffer->Create(InDevice, RenderMeshData);
}

void USkeletalMesh::EnsureMeshTrianglePickingBVHBuilt() const
{
    if (!SkeletalMeshAsset)
    {
        return;
    }

    MeshTrianglePickingBVH.EnsureBuilt(*SkeletalMeshAsset);
}

bool USkeletalMesh::RaycastMeshTrianglesWithBVHLocal(const FVector& LocalOrigin, const FVector& LocalDirection, FHitResult& OutHitResult) const
{
    if (!SkeletalMeshAsset)
    {
        return false;
    }

    EnsureMeshTrianglePickingBVHBuilt();
    return MeshTrianglePickingBVH.RaycastLocal(LocalOrigin, LocalDirection, *SkeletalMeshAsset, OutHitResult);
}