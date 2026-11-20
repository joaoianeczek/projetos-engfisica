"""Coleta dados IAQ do ESP32 via USB, salva CSV e gera gráficos.

Dependências: pip install pyserial pandas matplotlib
Exemplo: python coletar_iaq.py --porta COM8
Finalize com Ctrl+C para gerar os gráficos.
"""
import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

import pandas as pd
import matplotlib.pyplot as plt
import serial

COLUNAS = ["data_hora", "tempo_s", "iaq", "iaq_estatico", "bvoc_ppm",
           "temperatura_c", "umidade_pct", "acuracia", "estado"]


def gerar_graficos(arquivo: Path):
    dados = pd.read_csv(arquivo)
    if dados.empty:
        print("Nenhuma leitura válida foi registrada; sem gráfico.")
        return
    for coluna in ["tempo_s", "iaq", "iaq_estatico"]:
        dados[coluna] = pd.to_numeric(dados[coluna], errors="coerce")
    dados = dados.dropna(subset=["tempo_s", "iaq", "iaq_estatico"])
    if dados.empty:
        print("O CSV não contém valores numéricos válidos para os gráficos.")
        return

    plt.figure(figsize=(11, 5))
    plt.plot(dados["tempo_s"], dados["iaq"], label="IAQ (adaptativo)")
    plt.plot(dados["tempo_s"], dados["iaq_estatico"], label="IAQ estático")
    plt.axhline(100, linestyle="--", alpha=0.5, label="Limite bom/moderado")
    plt.axhline(200, linestyle="--", alpha=0.5, label="Limite moderado/ruim")
    plt.axhline(300, linestyle="--", alpha=0.5, label="Limite ruim/crítico")
    plt.xlabel("Tempo desde a primeira leitura válida (s)")
    plt.ylabel("Índice de qualidade do ar (IAQ)")
    plt.title("BME688 / BSEC: IAQ ao longo do tempo")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    imagem = arquivo.with_suffix(".png")
    plt.savefig(imagem, dpi=160)
    plt.show()
    print(f"Gráfico salvo em: {imagem}")


def main():
    parser = argparse.ArgumentParser(description="Coletor serial IAQ para ESP32/BME688")
    parser.add_argument("--porta", required=True, help="Ex.: COM8, COM12 ou /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pasta", default="dados_iaq")
    args = parser.parse_args()

    pasta = Path(args.pasta)
    pasta.mkdir(parents=True, exist_ok=True)
    arquivo = pasta / f"iaq_{datetime.now():%Y%m%d_%H%M%S}.csv"

    print(f"Conectando à porta {args.porta} ({args.baud} baud)...")
    try:
        with serial.Serial(args.porta, args.baud, timeout=1) as esp, \
                arquivo.open("w", newline="", encoding="utf-8") as f:
            gravador = csv.writer(f)
            gravador.writerow(COLUNAS)
            f.flush()
            print(f"Registrando em {arquivo}. Ctrl+C encerra e gera gráfico.")
            print("Feche o Monitor Serial do VS Code/Arduino para liberar a porta.")
            while True:
                bruto = esp.readline().decode("utf-8", errors="replace").strip()
                if not bruto:
                    continue
                if not bruto.startswith("DATA,"):
                    print(bruto)
                    continue
                campos = next(csv.reader([bruto]))
                if len(campos) != 9:
                    print(f"Linha ignorada (formato inválido): {bruto}")
                    continue
                try:
                    float(campos[1])
                    float(campos[2])
                    float(campos[3])
                    float(campos[4])
                    float(campos[5])
                    float(campos[6])
                    int(campos[7])
                except ValueError:
                    print(f"Linha ignorada (valores inválidos): {bruto}")
                    continue
                gravador.writerow([datetime.now().isoformat(timespec="seconds"), *campos[1:]])
                f.flush()
                print(f"t={campos[1]} s | IAQ={campos[2]} | sIAQ={campos[3]} | acc={campos[7]} | {campos[8]}")
    except KeyboardInterrupt:
        print("\nColeta encerrada pelo usuário.")
    except serial.SerialException as exc:
        print(f"Erro na porta serial: {exc}")
    finally:
        if arquivo.exists():
            gerar_graficos(arquivo)


if __name__ == "__main__":
    main()
