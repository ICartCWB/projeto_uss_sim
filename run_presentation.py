import sys
import time
from pyngrok import ngrok, conf
import config
import subprocess

def main():
    print("🚀 Preparando ambiente de apresentação USS...")

    # 1. Configuração do Ngrok
    if not config.NGROK_AUTH_TOKEN:
        print("❌ ERRO: 'NGROK_AUTH_TOKEN' não encontrado no .env ou config.py")
        print("   Obtenha seu token em: https://dashboard.ngrok.com/get-started/your-authtoken")
        return

    # Define o token na configuração global do pyngrok
    conf.get_default().auth_token = config.NGROK_AUTH_TOKEN
    
    # (Opcional) Define região, ex: 'sa' para South America (menor latência)
    # conf.get_default().region = "sa"

    try:
        # 2. Abre o Túnel na porta 8501 (Padrão do Streamlit)
        # Fecha túneis anteriores para evitar erros
        ngrok.kill()
        
        # Cria o túnel HTTP
        public_url = ngrok.connect(8501).public_url
        
        print("\n" + "="*60)
        print(f"✅ TÚNEL ATIVO! SUA APLICAÇÃO ESTÁ ONLINE:")
        print(f"🔗 Link Público: {public_url}")
        print("="*60 + "\n")
        
        # Pequena pausa para leitura
        time.sleep(3)

        # 3. Inicia o Streamlit
        print("▶️ Iniciando Streamlit App...\n")
        
        # Executa o comando 'streamlit run app.py' como um subprocesso
        # sys.executable garante que usamos o mesmo Python do ambiente virtual atual
        subprocess.run([sys.executable, "-m", "streamlit", "run", "app.py"])

    except KeyboardInterrupt:
        print("\n🛑 Encerrando apresentação e fechando túnel...")
        ngrok.kill()
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Erro inesperado: {e}")
        ngrok.kill()

if __name__ == "__main__":
    main()