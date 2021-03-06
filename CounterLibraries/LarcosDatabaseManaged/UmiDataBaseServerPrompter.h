#pragma once


using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;

namespace LarcosDatabaseManaged 
{

	/// <summary>
	/// Summary for UmiDataBaseServerPrompter
	///
	/// WARNING: If you change the name of this class, you will need to change the
	///          'Resource File Name' property for the managed resource compiler tool
	///          associated with all .resx files this class depends on.  Otherwise,
	///          the designers will not be able to interact properly with localized
	///          resources associated with this form.
	/// </summary>
	public ref class UmiDataBaseServerPrompter : public System::Windows::Forms::Form
	{
  private:
   UmiRunLog^              log;
   UmiDataBaseServerList^  servers; 


  private: System::Windows::Forms::Button^  ConnectButton;
           UmiDataBaseServer^               selectedServer;   // Will be initialized to current default server;

  public:
    property  UmiDataBaseServer^  SelectedServer  {UmiDataBaseServer^  get ()  {return  selectedServer;}}

	public:
		UmiDataBaseServerPrompter(UmiRunLog^  _log)
		{
      log = _log;
      servers = gcnew UmiDataBaseServerList (log);
      selectedServer = servers->GetDefaultServer ();

			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~UmiDataBaseServerPrompter()
		{
			if (components)
			{
				delete components;
			}
		}
  private: System::Windows::Forms::TextBox^  HostName;
  protected: 
  private: System::Windows::Forms::TextBox^  Description;
  private: System::Windows::Forms::Label^  DescriptionLabel;
  private: System::Windows::Forms::Label^  HostNameLabel;
  private: System::Windows::Forms::Label^  UserNameLabel;
  private: System::Windows::Forms::Label^  PasswordLabel;
  private: System::Windows::Forms::TextBox^  UserName;
  private: System::Windows::Forms::TextBox^  PassWord;

  private: System::Windows::Forms::Label^  DababaseLabel;
  private: System::Windows::Forms::TextBox^  DataBase;
  private: System::Windows::Forms::Label^  ServersLabel;
  private: System::Windows::Forms::ListBox^  Servers;
  private: System::Windows::Forms::Button^  CancelButton;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
      this->HostName = (gcnew System::Windows::Forms::TextBox());
      this->Description = (gcnew System::Windows::Forms::TextBox());
      this->DescriptionLabel = (gcnew System::Windows::Forms::Label());
      this->HostNameLabel = (gcnew System::Windows::Forms::Label());
      this->UserNameLabel = (gcnew System::Windows::Forms::Label());
      this->PasswordLabel = (gcnew System::Windows::Forms::Label());
      this->UserName = (gcnew System::Windows::Forms::TextBox());
      this->PassWord = (gcnew System::Windows::Forms::TextBox());
      this->DababaseLabel = (gcnew System::Windows::Forms::Label());
      this->DataBase = (gcnew System::Windows::Forms::TextBox());
      this->ServersLabel = (gcnew System::Windows::Forms::Label());
      this->Servers = (gcnew System::Windows::Forms::ListBox());
      this->CancelButton = (gcnew System::Windows::Forms::Button());
      this->ConnectButton = (gcnew System::Windows::Forms::Button());
      this->SuspendLayout();
      // 
      // HostName
      // 
      this->HostName->Location = System::Drawing::Point(116, 286);
      this->HostName->Name = L"HostName";
      this->HostName->Size = System::Drawing::Size(236, 20);
      this->HostName->TabIndex = 22;
      // 
      // Description
      // 
      this->Description->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->Description->Location = System::Drawing::Point(116, 256);
      this->Description->Name = L"Description";
      this->Description->Size = System::Drawing::Size(236, 22);
      this->Description->TabIndex = 21;
      // 
      // DescriptionLabel
      // 
      this->DescriptionLabel->AutoSize = true;
      this->DescriptionLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, 
        System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
      this->DescriptionLabel->Location = System::Drawing::Point(28, 260);
      this->DescriptionLabel->Name = L"DescriptionLabel";
      this->DescriptionLabel->Size = System::Drawing::Size(76, 16);
      this->DescriptionLabel->TabIndex = 2;
      this->DescriptionLabel->Text = L"Description";
      // 
      // HostNameLabel
      // 
      this->HostNameLabel->AutoSize = true;
      this->HostNameLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->HostNameLabel->Location = System::Drawing::Point(28, 290);
      this->HostNameLabel->Name = L"HostNameLabel";
      this->HostNameLabel->Size = System::Drawing::Size(76, 16);
      this->HostNameLabel->TabIndex = 3;
      this->HostNameLabel->Text = L"Host Name";
      // 
      // UserNameLabel
      // 
      this->UserNameLabel->AutoSize = true;
      this->UserNameLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->UserNameLabel->Location = System::Drawing::Point(28, 320);
      this->UserNameLabel->Name = L"UserNameLabel";
      this->UserNameLabel->Size = System::Drawing::Size(77, 16);
      this->UserNameLabel->TabIndex = 4;
      this->UserNameLabel->Text = L"User Name";
      // 
      // PasswordLabel
      // 
      this->PasswordLabel->AutoSize = true;
      this->PasswordLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->PasswordLabel->Location = System::Drawing::Point(28, 350);
      this->PasswordLabel->Name = L"PasswordLabel";
      this->PasswordLabel->Size = System::Drawing::Size(68, 16);
      this->PasswordLabel->TabIndex = 5;
      this->PasswordLabel->Text = L"Password";
      // 
      // UserName
      // 
      this->UserName->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->UserName->Location = System::Drawing::Point(116, 316);
      this->UserName->Name = L"UserName";
      this->UserName->Size = System::Drawing::Size(236, 22);
      this->UserName->TabIndex = 23;
      // 
      // PassWord
      // 
      this->PassWord->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->PassWord->Location = System::Drawing::Point(116, 346);
      this->PassWord->Name = L"PassWord";
      this->PassWord->Size = System::Drawing::Size(236, 22);
      this->PassWord->TabIndex = 24;
      // 
      // DababaseLabel
      // 
      this->DababaseLabel->AutoSize = true;
      this->DababaseLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->DababaseLabel->Location = System::Drawing::Point(28, 380);
      this->DababaseLabel->Name = L"DababaseLabel";
      this->DababaseLabel->Size = System::Drawing::Size(69, 16);
      this->DababaseLabel->TabIndex = 8;
      this->DababaseLabel->Text = L"DataBase";
      // 
      // DataBase
      // 
      this->DataBase->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->DataBase->Location = System::Drawing::Point(116, 376);
      this->DataBase->Name = L"DataBase";
      this->DataBase->Size = System::Drawing::Size(236, 22);
      this->DataBase->TabIndex = 25;
      // 
      // ServersLabel
      // 
      this->ServersLabel->AutoSize = true;
      this->ServersLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->ServersLabel->Location = System::Drawing::Point(28, 23);
      this->ServersLabel->Name = L"ServersLabel";
      this->ServersLabel->Size = System::Drawing::Size(55, 16);
      this->ServersLabel->TabIndex = 10;
      this->ServersLabel->Text = L"Servers";
      // 
      // Servers
      // 
      this->Servers->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
        static_cast<System::Byte>(0)));
      this->Servers->FormattingEnabled = true;
      this->Servers->ItemHeight = 16;
      this->Servers->Location = System::Drawing::Point(116, 23);
      this->Servers->Name = L"Servers";
      this->Servers->Size = System::Drawing::Size(235, 212);
      this->Servers->TabIndex = 20;
      this->Servers->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &UmiDataBaseServerPrompter::Servers_MouseDoubleClick);
      this->Servers->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &UmiDataBaseServerPrompter::Servers_MouseClick);
      this->Servers->SelectedIndexChanged += gcnew System::EventHandler(this, &UmiDataBaseServerPrompter::Servers_SelectedIndexChanged);
      // 
      // CancelButton
      // 
      this->CancelButton->Location = System::Drawing::Point(276, 419);
      this->CancelButton->Name = L"CancelButton";
      this->CancelButton->Size = System::Drawing::Size(75, 23);
      this->CancelButton->TabIndex = 27;
      this->CancelButton->Text = L"Cancel";
      this->CancelButton->UseVisualStyleBackColor = true;
      this->CancelButton->Click += gcnew System::EventHandler(this, &UmiDataBaseServerPrompter::CancelButton_Click);
      // 
      // ConnectButton
      // 
      this->ConnectButton->Location = System::Drawing::Point(31, 419);
      this->ConnectButton->Name = L"ConnectButton";
      this->ConnectButton->Size = System::Drawing::Size(75, 23);
      this->ConnectButton->TabIndex = 26;
      this->ConnectButton->Text = L"Connect";
      this->ConnectButton->UseVisualStyleBackColor = true;
      this->ConnectButton->Click += gcnew System::EventHandler(this, &UmiDataBaseServerPrompter::ConnectButton_Click);
      // 
      // UmiDataBaseServerPrompter
      // 
      this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
      this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
      this->ClientSize = System::Drawing::Size(410, 464);
      this->Controls->Add(this->ConnectButton);
      this->Controls->Add(this->CancelButton);
      this->Controls->Add(this->Servers);
      this->Controls->Add(this->ServersLabel);
      this->Controls->Add(this->DataBase);
      this->Controls->Add(this->DababaseLabel);
      this->Controls->Add(this->PassWord);
      this->Controls->Add(this->UserName);
      this->Controls->Add(this->PasswordLabel);
      this->Controls->Add(this->UserNameLabel);
      this->Controls->Add(this->HostNameLabel);
      this->Controls->Add(this->DescriptionLabel);
      this->Controls->Add(this->Description);
      this->Controls->Add(this->HostName);
      this->Name = L"UmiDataBaseServerPrompter";
      this->Text = L"UmiDataBaseServerPrompter";
      this->Load += gcnew System::EventHandler(this, &UmiDataBaseServerPrompter::UmiDataBaseServerPrompter_Load);
      this->FormClosing += gcnew System::Windows::Forms::FormClosingEventHandler(this, &UmiDataBaseServerPrompter::UmiDataBaseServerPrompter_FormClosing);
      this->ResumeLayout(false);
      this->PerformLayout();

    }
#pragma endregion


private: void  UpdateSceenFieldsFromSelectedEntry ()
         {
           //System::Windows::Forms::ListBox^  servers = (System::Windows::Forms::ListBox^)sender;
           int  selIdx = Servers->SelectedIndex;
           if  (selIdx < 0)
             return;

           selectedServer = servers->LookUpByDescription ((String^)(Servers->Items[selIdx]));
           if  (selectedServer == nullptr)
             return;

           Description->Text = selectedServer->Description;
           HostName->Text    = selectedServer->HostName;
           UserName->Text    = selectedServer->UserName;
           PassWord->Text    = selectedServer->PassWord;
           DataBase->Text    = selectedServer->DataBaseName;
         }




private: System::Void  Servers_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
         {
           //System::Windows::Forms::ListBox^  servers = (System::Windows::Forms::ListBox^)sender;
           UpdateSceenFieldsFromSelectedEntry ();
         }



private: System::Void  Servers_MouseDoubleClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
         {
           //System::Windows::Forms::ListBox^  servers = (System::Windows::Forms::ListBox^)sender;
           int  selIdx = Servers->SelectedIndex;
           if  (selIdx < 0)
             return;

           selectedServer = servers->LookUpByDescription ((String^)(Servers->Items[selIdx]));
           this->Close ();
         }


private: System::Void Servers_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e)
         {
           UpdateSceenFieldsFromSelectedEntry ();
         }


private: System::Void UmiDataBaseServerPrompter_Load(System::Object^  sender, System::EventArgs^  e) 
         {
           Servers->Items->Clear ();

           int  selectedIdx = -1;
           int  idx = 0;

           if  (servers != nullptr)
           {
             for each  (UmiDataBaseServer^ pdbs in servers)
             {
               if  (pdbs  == selectedServer)
                 selectedIdx = idx;

               Servers->Items->Add (pdbs->Description);
               idx++;
             }
           }

           if  (selectedIdx >= 0)
             Servers->SelectedIndex = selectedIdx;
         }


private: System::Void CancelButton_Click(System::Object^  sender, System::EventArgs^  e) 
         {
           selectedServer = nullptr;
           this->Close ();
         }


private: System::Void ConnectButton_Click(System::Object^  sender, System::EventArgs^  e) 
         {
           if  (String::IsNullOrEmpty (Description->Text))
             return;

           UmiDataBaseServer^  existingServer = servers->LookUpByDescription (Description->Text);
           if  (existingServer == nullptr)
           {
             // We are adding a new server
             UmiDataBaseServer^  newServer = gcnew UmiDataBaseServer ();
             servers->Add (newServer);
             existingServer = newServer;
           }

           existingServer->Description  = Description->Text;
           existingServer->HostName     = HostName->Text;
           existingServer->UserName     = UserName->Text;
           existingServer->PassWord     = PassWord->Text;
           existingServer->DataBaseName = DataBase->Text;

           servers->DefaultServerDescription = Description->Text;

           // We will try to connect first;  if we can connect will add to list of existing servers.
           LarcosDatabaseManaged::UmiDataBase^  newConn = gcnew UmiDataBase (existingServer, log);
           if  (newConn->Valid ())
           {
             //servers->Add (existingServer);
             servers->WriteConfigFile ();
             //delete  newConn;
             newConn = nullptr;
             selectedServer = existingServer;
             this->Close ();
           }
           else
           {
             System::Windows::Forms::MessageBox::Show ("Cound not make connection\n\n" + newConn->LastErrorDesc (), "Database Connection");
             //delete  newConn;
             newConn = nullptr;
           }
         }
private: System::Void UmiDataBaseServerPrompter_FormClosing(System::Object^  sender, System::Windows::Forms::FormClosingEventArgs^  e) {
           System::Windows::Forms::CloseReason  cr =  e->CloseReason;
           int  zed = 9090;
         }
};
}
