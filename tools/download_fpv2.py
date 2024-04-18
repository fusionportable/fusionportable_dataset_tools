#! /usr/bin/env python3

import gdown
import os

fp_dataset_folder = '/Rocket_ssd/dataset/FusionPortableV2_dataset_release'

dict_data_key = {
  # 'sensor_data/handheld_escalator00.7z' : '1UmTWl7mEH089t6jlsbUjPPqgKueX3IEo'
  # 'sensor_data/handheld_escalator01.7z' : '1sCg27gpMYbV1i_XzadRt14ffBrw6TYQW'
  'sensor_data/handheld_grass00.7z' : '1jTrnHkZEfJSieXe7JFDE-XjlQSHF8rR_',
  # 'sensor_data/handheld_room00.7z' : '17KpBjb0d4qqO7y77JunZe2bkpFCZ_KEu',
  # 'sensor_data/handheld_room01.7z' : '1RkqbfOWjGSl5_Wojz7pK02eanpp7Pvkn',
  # 'sensor_data/handheld_underground.7z' : '1BitdaufLVZ1t2ZKTBQ7KBNuZPza2Wln1',
  # 'sensor_data/legged_grass00.7z' : '1RtDS02rYlJaH55zJS-sfNOjPnNmNj1zI',
  # 'sensor_data/legged_grass01.7z' : '1KZXxai2ZrA5W9of4PbVpUiDQgVpvn82M',
  # 'sensor_data/legged_room00.7z' : '1DmBNOgKETClwOQaO9lMoypKhyDjJqc8b',
  # 'sensor_data/legged_transition00.7z' : '1PP_MG6b14Ne_boRkcfaCtDn0KJFxbIDe',
  # 'sensor_data/legged_underground00.7z' : '1znfUKifWUN2jZbaEAMl65DiHCovbReTB',
  # 'sensor_data/ugv_campus00.7z' : '1N3_JQwvk3VB1FwLitWfLL0GU2Qj45w1q',
  # 'sensor_data/ugv_campus01.7z' : '17mewakWMRY8pI4XbAEy45kK0sTDUGLfV',
  # 'sensor_data/ugv_parking00.7z' : '17pnn17Z0ms8qCYKC8VgJS5ypoS46lodA',
  # 'sensor_data/ugv_parking01.7z' : '1xA6vTnhdIf0BFG9_KL2oscAAUOL2L2VP',
  # 'sensor_data/ugv_parking02.7z' : '1v7qdXgV4s_zuVe26MLewlqEIl2w5x1p8',
  # 'sensor_data/ugv_parking03.7z' : '1Y5OTvDUa_dfaHYlsgmew5HhEaGp4rB0L',
  # 'sensor_data/ugv_transition00.7z' : '15ChHaSBrR05EL6X7W1VGSuo_zfebCvIm',
  # 'sensor_data/ugv_transition01.7z' : '1XZKnVSJKCoMo3Qr6wZDe2ouYLFUPc6fS',
  # 'sensor_data/vehicle_campus00.7z' : '1SnsSgb-kCbYxsNJJFpVcSlgrnfV0-Ocv',
  # 'sensor_data/vehicle_campus01.7z' : '1ul1g7MYSQwJHe_axChH3quCagIpymPrV',
  # 'sensor_data/vehicle_downhill00.7z' : '1p5dS4BzN3--Rf25cln8FJyq_wgA8dWog',
  # 'sensor_data/vehicle_highway00.7z' : '1bdhbJuXtlvyiQwRMPFS2Ja94Xy3itvJ6',
  # 'sensor_data/vehicle_highway01.7z' : '1q4vmwvEH4NNmznZpvK5Ba9PFLnXY2iLw',
  # 'sensor_data/vehicle_multilayer00.7z' : '1EtGRmmaTmtQFykxal8TqFg20LrZX8TSI',
  # 'sensor_data/vehicle_street00.7z' : '1WmVtx4e9nIXroIgUB3JSdvL1Ox4f0G14',
  # 'sensor_data/vehicle_tunnel00.7z' : '15ija4SEI6wfGPug0ZtCx5yf2KYDVR5In'
}

base_url = 'https://drive.google.com/uc?id={}'

if __name__ == '__main__':
  if not os.path.exists(fp_dataset_folder):
    os.makedirs(fp_dataset_folder)
    os.makedirs(os.path.join(fp_dataset_folder, 'sensor_data'))
    os.makedirs(os.path.join(fp_dataset_folder, 'evaluation'))
    os.makedirs(os.path.join(fp_dataset_folder, 'groundtruth'))

  for key, value in dict_data_key.items():
    print(key, value)
    output_path = os.path.join(fp_dataset_folder, key)
    url = base_url.format(value)
    gdown.download(url, output_path, quiet=False)
    # from google_drive_downloader import GoogleDriveDownloader as gdd
    # gdd.download_file_from_google_drive(file_id=value,
    #                                     dest_path=output_path,
    #                                     unzip=False)