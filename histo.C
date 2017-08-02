{
  std::ifstream f;
  f.open("hist.txt",std::ios::in);
  TH1F *h = new TH1F("histo","histo",1000,-4,6);
  float a,b;
  long long int i; 
  while(i<2000)
//   while(!f.eof())
  {
    i++;
    f >> a >> b;
//     std::cout << i << " " << a << std::endl;
    
//     if(!f.eof())
      h->Fill(a,b);
  }
  f.close();
  h->Draw();
}