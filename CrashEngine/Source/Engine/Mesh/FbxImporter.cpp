// FBX SDK based skeletal mesh importer.
#include "Mesh/FbxImporter.h"

#include "Core/Logging/LogMacros.h"
#include "Mesh/SkeletalMeshAsset.h"

#include <fbxsdk.h>

#include <algorithm>
#include <memory>

namespace
{
struct FFbxManagerDeleter
{
    void operator()(FbxManager* Manager) const
    {
        if (Manager)
        {
            Manager->Destroy();
        }
    }
};

struct FControlPointInfluence
{
    int32 BoneIndex = -1;
    float Weight = 0.0f;
};

struct FVertexBuildInfo
{
    int32 ControlPointIndex = -1;
};

FMatrix ToEngineMatrix(const FbxAMatrix& Matrix)
{
    FMatrix Result = FMatrix::Identity;
    for (int32 Row = 0; Row < 4; ++Row)
    {
        for (int32 Col = 0; Col < 4; ++Col)
        {
            Result.M[Row][Col] = static_cast<float>(Matrix.Get(Row, Col));
        }
    }
    return Result;
}

FString GetNodeName(FbxNode* Node)
{
    return Node && Node->GetName() ? FString(Node->GetName()) : FString();
}

bool IsSkeletonNode(FbxNode* Node)
{
    return Node && Node->GetNodeAttribute() && Node->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton;
}

void CollectSkeletonNodes(FbxNode* Node, TArray<FbxNode*>& OutNodes)
{
    if (!Node)
    {
        return;
    }

    if (IsSkeletonNode(Node))
    {
        OutNodes.push_back(Node);
    }

    const int32 ChildCount = Node->GetChildCount();
    for (int32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
    {
        CollectSkeletonNodes(Node->GetChild(ChildIndex), OutNodes);
    }
}

FbxNode* FindFirstMeshNode(FbxNode* Node)
{
    if (!Node)
    {
        return nullptr;
    }

    if (Node->GetMesh())
    {
        return Node;
    }

    const int32 ChildCount = Node->GetChildCount();
    for (int32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
    {
        if (FbxNode* MeshNode = FindFirstMeshNode(Node->GetChild(ChildIndex)))
        {
            return MeshNode;
        }
    }

    return nullptr;
}

template <typename ElementType, typename ValueType>
bool ReadLayerElement(const ElementType* Element, int32 ControlPointIndex, int32 PolygonVertexIndex, ValueType& OutValue)
{
    if (!Element)
    {
        return false;
    }

    int32 Index = -1;
    switch (Element->GetMappingMode())
    {
    case FbxGeometryElement::eByControlPoint:
        Index = ControlPointIndex;
        break;
    case FbxGeometryElement::eByPolygonVertex:
        Index = PolygonVertexIndex;
        break;
    default:
        return false;
    }

    if (Element->GetReferenceMode() == FbxGeometryElement::eIndexToDirect)
    {
        if (Index < 0 || Index >= Element->GetIndexArray().GetCount())
        {
            return false;
        }
        Index = Element->GetIndexArray().GetAt(Index);
    }

    if (Index < 0 || Index >= Element->GetDirectArray().GetCount())
    {
        return false;
    }

    OutValue = Element->GetDirectArray().GetAt(Index);
    return true;
}

FVector ToVector(const FbxVector4& Value)
{
    return FVector(static_cast<float>(Value[0]), static_cast<float>(Value[1]), static_cast<float>(Value[2]));
}

FVector2 ToVector2(const FbxVector2& Value)
{
    return FVector2(static_cast<float>(Value[0]), 1.0f - static_cast<float>(Value[1]));
}

FVector ReadNormal(FbxMesh* Mesh, int32 ControlPointIndex, int32 PolygonVertexIndex)
{
    FbxVector4 Normal(0.0, 0.0, 1.0, 0.0);
    ReadLayerElement(Mesh->GetElementNormal(0), ControlPointIndex, PolygonVertexIndex, Normal);
    FVector Result = ToVector(Normal);
    Result.Normalize();
    return Result;
}

FVector4 ReadTangent(FbxMesh* Mesh, int32 ControlPointIndex, int32 PolygonVertexIndex)
{
    FbxVector4 Tangent(1.0, 0.0, 0.0, 1.0);
    ReadLayerElement(Mesh->GetElementTangent(0), ControlPointIndex, PolygonVertexIndex, Tangent);
    FVector Tangent3 = ToVector(Tangent);
    Tangent3.Normalize();
    return FVector4(Tangent3, 1.0f);
}

FVector2 ReadUV(FbxMesh* Mesh, int32 PolygonIndex, int32 PolygonCornerIndex)
{
    FbxStringList UVSetNames;
    Mesh->GetUVSetNames(UVSetNames);
    if (UVSetNames.GetCount() <= 0)
    {
        return FVector2(0.0f, 0.0f);
    }

    FbxVector2 UV(0.0, 0.0);
    bool bUnmapped = false;
    if (Mesh->GetPolygonVertexUV(PolygonIndex, PolygonCornerIndex, UVSetNames[0], UV, bUnmapped) && !bUnmapped)
    {
        return ToVector2(UV);
    }

    return FVector2(0.0f, 0.0f);
}

void BuildSkeleton(FbxScene* Scene, FSkeletalMesh& OutMesh, TMap<FbxNode*, int32>& OutBoneIndexByNode)
{
    TArray<FbxNode*> SkeletonNodes;
    CollectSkeletonNodes(Scene->GetRootNode(), SkeletonNodes);

    OutMesh.Bones.clear();
    OutMesh.Bones.reserve(SkeletonNodes.size());
    OutBoneIndexByNode.clear();

    for (FbxNode* Node : SkeletonNodes)
    {
        const int32 BoneIndex = static_cast<int32>(OutMesh.Bones.size());
        OutBoneIndexByNode[Node] = BoneIndex;

        FSkeletonBone Bone;
        Bone.Name = GetNodeName(Node);
        Bone.GlobalBindPose = ToEngineMatrix(Node->EvaluateGlobalTransform());
        Bone.CurrentGlobalPose = Bone.GlobalBindPose;

        if (FbxNode* Parent = Node->GetParent())
        {
            const FMatrix ParentGlobal = ToEngineMatrix(Parent->EvaluateGlobalTransform());
            Bone.LocalBindPose = Bone.GlobalBindPose * ParentGlobal.GetInverse();
        }
        else
        {
            Bone.LocalBindPose = Bone.GlobalBindPose;
        }
        Bone.InverseBindPose = Bone.GlobalBindPose.GetInverse();

        OutMesh.Bones.push_back(Bone);
    }

    for (FbxNode* Node : SkeletonNodes)
    {
        const int32 BoneIndex = OutBoneIndexByNode[Node];
        FbxNode* Parent = Node->GetParent();
        while (Parent)
        {
            auto ParentIterator = OutBoneIndexByNode.find(Parent);
            if (ParentIterator != OutBoneIndexByNode.end())
            {
                OutMesh.Bones[BoneIndex].ParentIndex = ParentIterator->second;
                break;
            }
            Parent = Parent->GetParent();
        }
    }
}

void BuildControlPointInfluences(FbxMesh* Mesh,
                                 FSkeletalMesh& OutMesh,
                                 const TMap<FbxNode*, int32>& BoneIndexByNode,
                                 TArray<TArray<FControlPointInfluence>>& OutInfluences)
{
    const int32 ControlPointCount = Mesh->GetControlPointsCount();
    OutInfluences.clear();
    OutInfluences.resize(ControlPointCount);

    const int32 SkinCount = Mesh->GetDeformerCount(FbxDeformer::eSkin);
    for (int32 SkinIndex = 0; SkinIndex < SkinCount; ++SkinIndex)
    {
        FbxSkin* Skin = static_cast<FbxSkin*>(Mesh->GetDeformer(SkinIndex, FbxDeformer::eSkin));
        if (!Skin)
        {
            continue;
        }

        const int32 ClusterCount = Skin->GetClusterCount();
        for (int32 ClusterIndex = 0; ClusterIndex < ClusterCount; ++ClusterIndex)
        {
            FbxCluster* Cluster = Skin->GetCluster(ClusterIndex);
            FbxNode* LinkNode = Cluster ? Cluster->GetLink() : nullptr;
            auto BoneIterator = BoneIndexByNode.find(LinkNode);
            if (BoneIterator == BoneIndexByNode.end())
            {
                continue;
            }

            const int32 BoneIndex = BoneIterator->second;

            FbxAMatrix LinkBindMatrix;
            Cluster->GetTransformLinkMatrix(LinkBindMatrix);
            OutMesh.Bones[BoneIndex].GlobalBindPose = ToEngineMatrix(LinkBindMatrix);
            OutMesh.Bones[BoneIndex].CurrentGlobalPose = OutMesh.Bones[BoneIndex].GlobalBindPose;
            OutMesh.Bones[BoneIndex].InverseBindPose = OutMesh.Bones[BoneIndex].GlobalBindPose.GetInverse();

            const int32* ControlPointIndices = Cluster->GetControlPointIndices();
            const double* ControlPointWeights = Cluster->GetControlPointWeights();
            const int32 InfluenceCount = Cluster->GetControlPointIndicesCount();

            for (int32 InfluenceIndex = 0; InfluenceIndex < InfluenceCount; ++InfluenceIndex)
            {
                const int32 ControlPointIndex = ControlPointIndices[InfluenceIndex];
                if (ControlPointIndex < 0 || ControlPointIndex >= ControlPointCount)
                {
                    continue;
                }

                FControlPointInfluence Influence;
                Influence.BoneIndex = BoneIndex;
                Influence.Weight = static_cast<float>(ControlPointWeights[InfluenceIndex]);
                OutInfluences[ControlPointIndex].push_back(Influence);
            }
        }
    }
}

void BuildSkinWeightsForVertices(const TArray<FVertexBuildInfo>& VertexInfos,
                                 const TArray<TArray<FControlPointInfluence>>& ControlPointInfluences,
                                 FSkeletalMesh& OutMesh)
{
    OutMesh.SkinWeights.clear();
    OutMesh.SkinWeights.resize(VertexInfos.size());

    for (uint32 VertexIndex = 0; VertexIndex < VertexInfos.size(); ++VertexIndex)
    {
        FSkinWeight Weight;
        const int32 ControlPointIndex = VertexInfos[VertexIndex].ControlPointIndex;
        if (ControlPointIndex >= 0 && ControlPointIndex < static_cast<int32>(ControlPointInfluences.size()))
        {
            TArray<FControlPointInfluence> Influences = ControlPointInfluences[ControlPointIndex];
            std::sort(Influences.begin(), Influences.end(), [](const FControlPointInfluence& A, const FControlPointInfluence& B)
                      { return A.Weight > B.Weight; });

            const int32 Count = (std::min)(static_cast<int32>(Influences.size()), MAX_BONE_INFLUENCES);
            for (int32 InfluenceIndex = 0; InfluenceIndex < Count; ++InfluenceIndex)
            {
                Weight.BoneIndices[InfluenceIndex] = Influences[InfluenceIndex].BoneIndex;
                Weight.BoneWeights[InfluenceIndex] = Influences[InfluenceIndex].Weight;
            }
        }

        Weight.Normalize();
        OutMesh.SkinWeights[VertexIndex] = Weight;
    }
}

bool BuildMeshGeometry(FbxMesh* Mesh, FSkeletalMesh& OutMesh, TArray<FVertexBuildInfo>& OutVertexInfos)
{
    if (!Mesh)
    {
        return false;
    }

    if (!Mesh->GetElementTangent(0))
    {
        Mesh->GenerateTangentsData(0, true);
    }

    OutMesh.RefVertices.clear();
    OutMesh.SkinnedVertices.clear();
    OutMesh.Indices.clear();
    OutMesh.Sections.clear();
    OutVertexInfos.clear();

    const int32 PolygonCount = Mesh->GetPolygonCount();
    int32 PolygonVertexIndex = 0;

    for (int32 PolygonIndex = 0; PolygonIndex < PolygonCount; ++PolygonIndex)
    {
        const int32 PolygonSize = Mesh->GetPolygonSize(PolygonIndex);
        if (PolygonSize != 3)
        {
            PolygonVertexIndex += PolygonSize;
            continue;
        }

        uint32 TriangleIndices[3] = {};
        for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
        {
            const int32 ControlPointIndex = Mesh->GetPolygonVertex(PolygonIndex, CornerIndex);
            const FbxVector4 Position = Mesh->GetControlPointAt(ControlPointIndex);

            FVertexPNCT_T Vertex;
            Vertex.Position = ToVector(Position);
            Vertex.Normal = ReadNormal(Mesh, ControlPointIndex, PolygonVertexIndex);
            Vertex.Color = FVector4(1.0f, 1.0f, 1.0f, 1.0f);
            Vertex.UV = ReadUV(Mesh, PolygonIndex, CornerIndex);
            Vertex.Tangent = ReadTangent(Mesh, ControlPointIndex, PolygonVertexIndex);

            TriangleIndices[CornerIndex] = static_cast<uint32>(OutMesh.RefVertices.size());
            OutMesh.RefVertices.push_back(Vertex);

            FVertexBuildInfo BuildInfo;
            BuildInfo.ControlPointIndex = ControlPointIndex;
            OutVertexInfos.push_back(BuildInfo);

            ++PolygonVertexIndex;
        }

        OutMesh.Indices.push_back(TriangleIndices[0]);
        OutMesh.Indices.push_back(TriangleIndices[2]);
        OutMesh.Indices.push_back(TriangleIndices[1]);
    }

    OutMesh.SkinnedVertices = OutMesh.RefVertices;

    if (!OutMesh.Indices.empty())
    {
        FStaticMeshSection Section;
        Section.MaterialIndex = -1;
        Section.MaterialSlotName = "Default";
        Section.FirstIndex = 0;
        Section.NumTriangles = static_cast<uint32>(OutMesh.Indices.size() / 3);
        OutMesh.Sections.push_back(Section);
    }

    OutMesh.CacheBounds();
    return !OutMesh.RefVertices.empty() && !OutMesh.Indices.empty();
}
} // namespace

bool FFbxImporter::ImportSkeletalMesh(const FString& FbxFilePath, FSkeletalMesh& OutMesh)
{
    std::unique_ptr<FbxManager, FFbxManagerDeleter> Manager(FbxManager::Create());
    if (!Manager)
    {
        UE_LOG(FbxImporter, Error, "Failed to create FbxManager.");
        return false;
    }

    FbxIOSettings* IOSettings = FbxIOSettings::Create(Manager.get(), IOSROOT);
    Manager->SetIOSettings(IOSettings);

    FbxScene* Scene = FbxScene::Create(Manager.get(), "ImportScene");
    FbxImporter* Importer = FbxImporter::Create(Manager.get(), "Importer");
    if (!Importer || !Scene)
    {
        UE_LOG(FbxImporter, Error, "Failed to create FBX scene/importer.");
        return false;
    }

    if (!Importer->Initialize(FbxFilePath.c_str(), -1, Manager->GetIOSettings()))
    {
        UE_LOG(FbxImporter, Error, "Failed to initialize FBX importer. File=%s Error=%s",
               FbxFilePath.c_str(), Importer->GetStatus().GetErrorString());
        Importer->Destroy();
        return false;
    }

    if (!Importer->Import(Scene))
    {
        UE_LOG(FbxImporter, Error, "Failed to import FBX scene. File=%s Error=%s",
               FbxFilePath.c_str(), Importer->GetStatus().GetErrorString());
        Importer->Destroy();
        return false;
    }
    Importer->Destroy();

    FbxGeometryConverter GeometryConverter(Manager.get());
    GeometryConverter.Triangulate(Scene, true);
    FbxAxisSystem::DirectX.ConvertScene(Scene);

    FbxNode* MeshNode = FindFirstMeshNode(Scene->GetRootNode());
    FbxMesh* Mesh = MeshNode ? MeshNode->GetMesh() : nullptr;
    if (!Mesh)
    {
        UE_LOG(FbxImporter, Error, "No mesh node found in FBX: %s", FbxFilePath.c_str());
        return false;
    }

    OutMesh = FSkeletalMesh();
    OutMesh.PathFileName = FbxFilePath;

    TMap<FbxNode*, int32> BoneIndexByNode;
    BuildSkeleton(Scene, OutMesh, BoneIndexByNode);
    if (OutMesh.Bones.empty())
    {
        UE_LOG(FbxImporter, Warning, "No skeleton bones found in FBX: %s", FbxFilePath.c_str());
    }

    TArray<FVertexBuildInfo> VertexInfos;
    if (!BuildMeshGeometry(Mesh, OutMesh, VertexInfos))
    {
        UE_LOG(FbxImporter, Error, "Failed to build skeletal mesh geometry: %s", FbxFilePath.c_str());
        return false;
    }

    TArray<TArray<FControlPointInfluence>> ControlPointInfluences;
    BuildControlPointInfluences(Mesh, OutMesh, BoneIndexByNode, ControlPointInfluences);
    BuildSkinWeightsForVertices(VertexInfos, ControlPointInfluences, OutMesh);

    UE_LOG(FbxImporter, Info, "FBX skeletal mesh imported. File=%s Vertices=%u Indices=%u Bones=%u",
           FbxFilePath.c_str(),
           static_cast<uint32>(OutMesh.RefVertices.size()),
           static_cast<uint32>(OutMesh.Indices.size()),
           static_cast<uint32>(OutMesh.Bones.size()));

    return true;
}
