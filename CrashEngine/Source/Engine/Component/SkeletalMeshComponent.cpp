#include "Component/SkeletalMeshComponent.h"
#include <algorithm>
#include <cmath>
#include "Object/ObjectFactory.h"
#include "Core/PropertyTypes.h"
#include "Collision/RayUtils.h"
#include "Mesh/SkeletalMeshAsset.h"
#include "Engine/Runtime/Engine.h"
#include "Materials/MaterialManager.h"
#include "Render/Resources/Shaders/ShaderManager.h"
#include "Texture/Texture2D.h"
// #include "Render/Scene/Proxies/Primitive/SkeletalMeshSceneProxy.h"
#include "Serialization/Archive.h"

IMPLEMENT_CLASS(USkeletalMeshComponent, UMeshComponent)

/*FPrimitiveProxy* USkeletalMeshComponent::CreateSceneProxy()
{
    return new FSkeletalMeshSceneProxy(this);
}*/

void USkeletalMeshComponent::SetSkeletalMesh(USkeletalMesh* InMesh)
{
    SkeletalMesh = InMesh;
    if (InMesh)
    {
        SkeletalMeshPath = InMesh->GetAssetPathFileName();
        const TArray<FMeshMaterial>& DefaultMaterials = SkeletalMesh->GetMaterials();

        OverrideMaterials.resize(DefaultMaterials.size());
        MaterialSlots.resize(DefaultMaterials.size());

        for (int32 i = 0; i < (int32)DefaultMaterials.size(); ++i)
        {
            OverrideMaterials[i] = DefaultMaterials[i].MaterialInterface;

            if (OverrideMaterials[i])
                MaterialSlots[i].Path = OverrideMaterials[i]->GetAssetPathFileName();
            else
                MaterialSlots[i].Path = "None";
        }

        if (FSkeletalMesh* Asset = SkeletalMesh->GetSkeletalMeshAsset())
        {
            ComponentSpaceTransforms.resize(Asset->Bones.size());
            RefreshComponentSpaceTransforms();
        }
    }
    else
    {
        SkeletalMeshPath = "None";
        OverrideMaterials.clear();
        MaterialSlots.clear();
        ComponentSpaceTransforms.clear();
    }
    CacheLocalBounds();
    MarkRenderStateDirty();
    MarkWorldBoundsDirty();
}

void USkeletalMeshComponent::CacheLocalBounds()
{
    bHasValidBounds = false;
    if (!SkeletalMesh)
        return;
    FSkeletalMesh* Asset = SkeletalMesh->GetSkeletalMeshAsset();
    if (!Asset || Asset->Vertices.empty())
        return;

    if (!Asset->bBoundsValid)
    {
        Asset->CacheBounds();
    }

    CachedLocalCenter = Asset->BoundsCenter;
    CachedLocalExtent = Asset->BoundsExtent;
    bHasValidBounds = Asset->bBoundsValid;
}

USkeletalMesh* USkeletalMeshComponent::GetSkeletalMesh() const
{
    return SkeletalMesh;
}

void USkeletalMeshComponent::SetMaterial(int32 ElementIndex, UMaterial* InMaterial)
{
    if (ElementIndex >= 0 && ElementIndex < static_cast<int32>(OverrideMaterials.size()))
    {
        OverrideMaterials[ElementIndex] = InMaterial;

        if (ElementIndex < static_cast<int32>(MaterialSlots.size()))
        {
            MaterialSlots[ElementIndex].Path = InMaterial ? InMaterial->GetAssetPathFileName() : "None";
        }
        MarkProxyDirty(ESceneProxyDirtyFlag::Material);
    }
}

UMaterial* USkeletalMeshComponent::GetMaterial(int32 ElementIndex) const
{
    if (ElementIndex >= 0 && ElementIndex < static_cast<int32>(OverrideMaterials.size()))
    {
        return OverrideMaterials[ElementIndex];
    }
    return nullptr;
}

FMeshBuffer* USkeletalMeshComponent::GetMeshBuffer() const
{
    // Skeletal meshes currently use FDynamicMeshBuffer, which is not compatible
    // with the legacy FMeshBuffer render path until a skeletal scene proxy exists.
    return nullptr;
}

FDynamicMeshBuffer* USkeletalMeshComponent::GetDynamicMeshBuffer() const
{
    if (!SkeletalMesh)
        return nullptr;

    FSkeletalMesh* Asset = SkeletalMesh->GetSkeletalMeshAsset();
    if (!Asset || !Asset->RenderBuffer || !Asset->RenderBuffer->IsValid())
        return nullptr;

    return Asset->RenderBuffer.get();
}

FMeshDataView USkeletalMeshComponent::GetMeshDataView() const
{
    if (!SkeletalMesh)
        return {};
    FSkeletalMesh* Asset = SkeletalMesh->GetSkeletalMeshAsset();
    if (!Asset || Asset->Vertices.empty())
        return {};

    FMeshDataView View;
    View.VertexData = Asset->Vertices.data();
    View.VertexCount = (uint32)Asset->Vertices.size();
    View.Stride = sizeof(FSkeletalMeshVertex);
    View.IndexData = Asset->Indices.data();
    View.IndexCount = (uint32)Asset->Indices.size();
    return View;
}

void USkeletalMeshComponent::UpdateWorldAABB() const
{
    if (!bHasValidBounds)
    {
        UPrimitiveComponent::UpdateWorldAABB();
        return;
    }

    FVector WorldCenter = CachedWorldMatrix.TransformPositionWithW(CachedLocalCenter);

    float Ex = std::abs(CachedWorldMatrix.M[0][0]) * CachedLocalExtent.X + std::abs(CachedWorldMatrix.M[1][0]) * CachedLocalExtent.Y + std::abs(CachedWorldMatrix.M[2][0]) * CachedLocalExtent.Z;
    float Ey = std::abs(CachedWorldMatrix.M[0][1]) * CachedLocalExtent.X + std::abs(CachedWorldMatrix.M[1][1]) * CachedLocalExtent.Y + std::abs(CachedWorldMatrix.M[2][1]) * CachedLocalExtent.Z;
    float Ez = std::abs(CachedWorldMatrix.M[0][2]) * CachedLocalExtent.X + std::abs(CachedWorldMatrix.M[1][2]) * CachedLocalExtent.Y + std::abs(CachedWorldMatrix.M[2][2]) * CachedLocalExtent.Z;

    WorldAABBMinLocation = WorldCenter - FVector(Ex, Ey, Ez);
    WorldAABBMaxLocation = WorldCenter + FVector(Ex, Ey, Ez);
    bWorldAABBDirty = false;
    bHasValidWorldAABB = true;
}

bool USkeletalMeshComponent::LineTraceComponent(const FRay& Ray, FHitResult& OutHitResult)
{
    if (!SkeletalMesh)
        return false;

    const FMatrix& WorldMatrix = GetWorldMatrix();
    const FMatrix& WorldInverse = GetWorldInverseMatrix();

    FVector LocalOrigin = WorldInverse.TransformPositionWithW(Ray.Origin);
    FVector LocalDirection = WorldInverse.TransformVector(Ray.Direction);
    LocalDirection.Normalize();

    if (SkeletalMesh->RaycastMeshTrianglesWithBVHLocal(LocalOrigin, LocalDirection, OutHitResult))
    {
        const FVector LocalHitPoint = LocalOrigin + LocalDirection * OutHitResult.Distance;
        const FVector WorldHitPoint = WorldMatrix.TransformPositionWithW(LocalHitPoint);
        OutHitResult.Distance = FVector::Distance(Ray.Origin, WorldHitPoint);
        OutHitResult.HitComponent = this;
        return true;
    }

    return false;
}

void USkeletalMeshComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction& ThisTickFunction)
{
    UMeshComponent::TickComponent(DeltaTime, TickType, ThisTickFunction);

    RefreshComponentSpaceTransforms();
}

void USkeletalMeshComponent::RefreshComponentSpaceTransforms()
{
    if (!SkeletalMesh)
    {
        ComponentSpaceTransforms.clear();
        return;
    }

    FSkeletalMesh* Asset = SkeletalMesh->GetSkeletalMeshAsset();
    if (!Asset)
    {
        ComponentSpaceTransforms.clear();
        return;
    }

    if (ComponentSpaceTransforms.size() != Asset->Bones.size())
    {
        ComponentSpaceTransforms.resize(Asset->Bones.size());
    }

    for (size_t i = 0; i < Asset->Bones.size(); ++i)
    {
        const FSkeletalBone& Bone = Asset->Bones[i];
        const FMatrix LocalMatrix = Bone.RefLocalTransform.ToMatrix();
        if (Bone.ParentIndex >= 0 && Bone.ParentIndex < static_cast<int32>(i))
        {
            ComponentSpaceTransforms[i] = LocalMatrix * ComponentSpaceTransforms[Bone.ParentIndex];
        }
        else
        {
            ComponentSpaceTransforms[i] = LocalMatrix;
        }
    }
}

static FArchive& operator<<(FArchive& Ar, FMaterialSlot& Slot)
{
    Ar << Slot.Path;
    return Ar;
}

void USkeletalMeshComponent::Serialize(FArchive& Ar)
{
    UMeshComponent::Serialize(Ar);
    Ar << SkeletalMeshPath;
    Ar << MaterialSlots;
}

void USkeletalMeshComponent::PostDuplicate()
{
    UMeshComponent::PostDuplicate();

    if (!SkeletalMeshPath.empty() && SkeletalMeshPath != "None")
    {
        USkeletalMesh* Loaded = nullptr;
        if (Loaded)
        {
            TArray<FMaterialSlot> SavedSlots = MaterialSlots;
            SetSkeletalMesh(Loaded);

            for (int32 i = 0; i < (int32)MaterialSlots.size() && i < (int32)SavedSlots.size(); ++i)
            {
                MaterialSlots[i] = SavedSlots[i];
                const FString& MatPath = MaterialSlots[i].Path;
                if (MatPath.empty() || MatPath == "None")
                {
                    OverrideMaterials[i] = nullptr;
                }
                else
                {
                    UMaterial* LoadedMat = FMaterialManager::Get().GetOrCreateStaticMeshMaterial(MatPath);
                    OverrideMaterials[i] = LoadedMat;
                }
            }
        }
    }

    CacheLocalBounds();
    MarkRenderStateDirty();
    MarkWorldBoundsDirty();
}

void USkeletalMeshComponent::GetEditableProperties(TArray<FPropertyDescriptor>& OutProps)
{
    UPrimitiveComponent::GetEditableProperties(OutProps);
    OutProps.push_back({ "Skeletal Mesh", EPropertyType::String, &SkeletalMeshPath });

    for (int32 i = 0; i < (int32)MaterialSlots.size(); ++i)
    {
        FPropertyDescriptor Desc;
        Desc.Name = "Element " + std::to_string(i);
        Desc.Type = EPropertyType::MaterialSlot;
        Desc.ValuePtr = &MaterialSlots[i];
        OutProps.push_back(Desc);
    }
}

void USkeletalMeshComponent::PostEditProperty(const char* PropertyName)
{
    UPrimitiveComponent::PostEditProperty(PropertyName);

    if (strcmp(PropertyName, "Skeletal Mesh") == 0)
    {
        if (SkeletalMeshPath.empty() || SkeletalMeshPath == "None")
        {
            SetSkeletalMesh(nullptr);
        }
        else
        {
            USkeletalMesh* Loaded = nullptr;
            SetSkeletalMesh(Loaded);
        }
        CacheLocalBounds();
        MarkWorldBoundsDirty();
    }

    if (strncmp(PropertyName, "Element ", 8) == 0)
    {
        int32 Index = atoi(&PropertyName[8]);

        if (Index >= 0 && Index < (int32)MaterialSlots.size())
        {
            FString NewMatPath = MaterialSlots[Index].Path;

            if (NewMatPath == "None" || NewMatPath.empty())
            {
                SetMaterial(Index, nullptr);
            }
            else
            {
                UMaterial* LoadedMat = FMaterialManager::Get().GetOrCreateStaticMeshMaterial(NewMatPath);
                if (LoadedMat)
                {
                    SetMaterial(Index, LoadedMat);
                }
            }
        }
    }
}
