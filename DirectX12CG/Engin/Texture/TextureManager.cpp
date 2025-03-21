#include "TextureManager.h"

using namespace MCB;
using namespace std;

//int32_t MCB::TextureManager::LoadTexture(const wchar_t* FileName, uint16_t int32_t incrementNum)
//{
//	std::unique_ptr<TextureCell> temp = make_unique<TextureCell>();
//	temp->texture->CreateTexture(FileName, incrementNum);
//	textures.push_back(move(temp));
//}
//
//int32_t MCB::TextureManager::LoadTexture(const std::string& directoryPath, const std::string& filename, uint16_t int32_t incrementNum)
//{
//	std::unique_ptr<TextureCell> temp = make_unique<TextureCell>();
//	temp->texture->CreateTexture(directoryPath, filename,incrementNum);
//	textures.push_back(move(temp));
//}


TextureCell* MCB::TextureManager::LoadTexture(const wchar_t* FileName)
{
	std::unique_ptr<TextureCell> temp = make_unique<TextureCell>();
	uint16_t tempin = 0;
	for (auto& itr : textures_)
	{
		if (tempin == itr->texture->incrementNum_)
		{
			tempin++;
			continue;
		}
	}

	temp->texture->CreateTexture(FileName,tempin);
	textures_.push_back(move(temp));
	//assert(textures.size() < 20);
	//texincrement.push_back(tempin);
	return textures_.rbegin()->get();
}

TextureCell* MCB::TextureManager::LoadTexture(const std::string& directoryPath, const std::string& filename)
{
	std::unique_ptr<TextureCell> temp = make_unique<TextureCell>();
	uint16_t tempin = 0;
	for (auto& itr : textures_)
	{
		if (tempin == itr->texture->incrementNum_)
		{
			tempin++;
			continue;
		}
	}
		temp->texture->CreateTexture(directoryPath, filename, tempin);
		textures_.push_back(move(temp));
		//assert(textures.size() < 20);

	//texincrement.push_back(tempin);
	return textures_.rbegin()->get();
}

TextureCell* MCB::TextureManager::CreateNoTextureFileIsTexture(bool postEffect)
{
	std::unique_ptr<TextureCell> temp = make_unique<TextureCell>();
	uint16_t tempin = 0;
	for (auto& itr : textures_)
	{
		if (tempin == itr->texture->incrementNum_)
		{
			tempin++;
			continue;
		}
	}
	temp->texture->CreateNoTextureFileIsTexture(tempin, postEffect);
	textures_.push_back(move(temp));
	//assert(textures.size() < 20);
	//texincrement.push_back(tempin);
	return textures_.rbegin()->get();
}

void MCB::TextureManager::Clear()
{
	textures_.clear();
}

void MCB::TextureManager::Erase()
{
	//for (auto& itr : textures)
	//{
	//	std::remove_if(texincrement.begin(), texincrement.end(), [&itr](int32_t itr2) { return (itr2 == itr.get()->texture.get()->incrementNum) && itr.get()->deleteFlag; });
	//}
	textures_.remove_if([](std::unique_ptr<TextureCell>& itr) { return itr->free; });
}

//Texture* MCB::TextureManager::GetTexture(uint16_t int32_t incrementNum)
//{
//	//return textures[incrementNum]->texture.get();
//}

//void MCB::TextureManager::SetDelete(int32_t index)
//{
//	//textures[index]->free = true;
//	//textures[index]->texture.reset();
//}


TextureManager* MCB::TextureManager::GetInstance()
{
	static TextureManager instance;// = new TextureManager();
	return &instance;
}

MCB::TextureManager::~TextureManager()
{
}

MCB::TextureManager::TextureManager()
{
}

void MCB::TextureManager::DeleteInstace()
{
	//delete TextureManager::GetInstance();
}

MCB::TextureCell::TextureCell()
{
	texture = std::make_unique<Texture>();
}
