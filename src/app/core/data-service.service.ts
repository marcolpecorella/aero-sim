// data.service.ts
import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class DataService {
  private apiUrl = 'http://localhost:8080/api/data';

  constructor(private http: HttpClient) { }

  postData(data: any): Observable<any> {
    return this.http.post<any>(this.apiUrl+"/position", data);
  }

  getFixes() {
    console.log('Getting fixes');
    return this.http.get<any>(`${this.apiUrl}/fixes`);
  }

  getSectors() {
    return this.http.get<any>(`${this.apiUrl}/sectors`);
  }

  getData(): Observable<any> {
    return this.http.get<any>(`${this.apiUrl}/fixes`);
  }
}
